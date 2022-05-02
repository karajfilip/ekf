#!/usr/bin/env python

import smach

from states import *
from assembly_kitting_states import *

if __name__ == '__main__':

    asm = smach.StateMachine(outcomes=['finished'], input_keys=['task', 'kittingtask'])

    with asm:
        smach.StateMachine.add('CHECKGRIPPER', CheckGripper(), transitions={'next':'SENDGANTRY', 'changegripper':'CHANGEGRIP'}, remapping={'gripper':'gripper'})
        smach.StateMachine.add('CHANGEGRIP', GetGripper(),  transitions={'gripperon':'SENDGANTRY'}, remapping={'gripper':'gripper'})
        smach.StateMachine.add('SENDGANTRY', SendGantry(), transitions={'arrived':'CHECKPART'}, remapping={'task':'task'})
        smach.StateMachine.add('CHECKPART', CheckPart(), transitions={'noParts':'SUBMITASSEMBLY', 'newPart':'FINDPART'}, remapping={'task':'task', 'part':'part'})
        smach.StateMachine.add('SUBMITASSEMBLY', SubmitAssemblyShipment(), transitions={'success':'finished'}, remapping={'task':'task'})
        smach.StateMachine.add('FINDPART', FindPartOnTray(), transitions={'found':'GANTRYMOVEPART'}, remapping={'task':'task', 'partcurrentposition': 'partcurrentposition'})
        smach.StateMachine.add('GANTRYMOVEPART', GantryMovePart(), transitions={'moved':'CHECKPART'}, remapping={'partcurrentposition':'partcurrentposition', 'part':'part'})

    ksm = smach.StateMachine(outcomes=['finished'], input_keys=['task', 'faultybinposition'])
    with ksm:
        smach.StateMachine.add('CHECKAGV', CheckAGV(), transitions={'agvatks':'CHECKTRAY', 'agvnotatks':'MOVEAGVTOKS'}, remapping={'task':'kittingtask'})
        smach.StateMachine.add('MOVEAGVTOKS', SendAGV(), transitions={'agvatks':'CHECKTRAY'}, remapping={'task':'kittingtask'})
        smach.StateMachine.add('CHECKTRAY', CheckMoveableTray(), transitions={'changegripper':'GETGRIPPER', 'changetray':'GETTRAY'}, remapping={'task':'kittingtask', 'gripper':'gripper'})
        smach.StateMachine.add('GETGRIPPER', GetGripper(), transitions={'gripperon':'GETTRAY'}, remapping={'gripper':'gripper'})
        smach.StateMachine.add('GETTRAY', GantryGetTray(), transitions={'trayon':'CHECKKITTINGPART'}, remapping={'task':'kittingtask'})
        smach.StateMachine.add('CHECKKITTINGPART', CheckPart(), transitions={'noParts':'SUBMITKITTING', 'newPart':'FINDPARTINENV'}, remapping={'task':'kittingtask', 'part':'part'})
        smach.StateMachine.add('FINDPARTINENV', FindPartInEnvironment(), transitions={'found':'KITTINGPICKANDPLACE'}, remapping={'part':'part', 'partcurrentposition':'partcurrentposition', 'partposition':'partposition'})
        smach.StateMachine.add('KITTINGPICKANDPLACE', KittingRobotPickAndPlace(), transitions={'success':'CHECKFAULTY'}, remapping={'partposition':'partposition', 'partcurrentposition':'partcurrentposition'})
        smach.StateMachine.add('CHECKFAULTY', CheckFaulty(), transitions={'faulty':'FAULTYPICKANDPLACE', 'notfaulty':'CHECKKITTINGPART'}, remapping={'part':'part'})
        smach.StateMachine.add('FAULTYPICKANDPLACE', KittingRobotPickAndPlace(), transitions={'success':'FINDPARTINENV'}, remapping={'partposition':'faultybinposition', 'partcurrentposition':'partposition'})
        smach.StateMachine.add('SUBMITKITTING', SubmitKittingShipment(), transitions={'success':'finished'}, remapping={'task':'kittingtask'})

    sm = smach.StateMachine(outcomes=['end'], input_keys=['faultybinposition'])

    with sm:

        smach.StateMachine.add('START', StartCompetition(), transitions={'success':'CHECKORDERS'}, remapping={'interrupted':'interrupted'})
        smach.StateMachine.add('CHECKORDERS', CheckOrders(), transitions={'complete':'end', 'nextOrder':'CHECKTASKS', 'highPriorityOrder':'HPCHECKTASKS'}, remapping={'nextOrder':'order', 'interrupted':'interrupted'})
        smach.StateMachine.add('CHECKTASKS', CheckTasks(), transitions={'kitting':'CKITTING', 'assembly':'CASSEMBLY', 'complete':'CHECKORDERS'}, remapping={'order':'order', 'task':'task'})

        concurent_assembly = smach.Concurrence(outcomes=['interrupted', 'complete'], default_outcome='complete', input_keys=['task'], output_keys=['nextOrder'], outcome_map={'interrupted':{'ORDERS':'highPriorityOrder'}, 'complete':{'ASSEMBLY':'finished'}})

        with concurent_assembly:
            smach.Concurrence.add('ASSEMBLY',  asm, remapping={'task':'task', 'kittingtask':'kittingtask'})
            smach.Concurrence.add('ORDERS', CheckOrders(), remapping={'nextOrder':'nextOrder', 'interrupted':'interrupted'})
        
        smach.StateMachine.add('CASSEMBLY', concurent_assembly, transitions={'complete':'CHECKORDERS', 'interrupted':'HPCHECKTASKS'}, remapping={'task':'task', 'nextOrder':'nextOrder'})
        
        concurent_kitting = smach.Concurrence(outcomes=['interrupted', 'complete'], default_outcome='complete', input_keys=['task'], output_keys=['nextOrder'], outcome_map={'interrupted':{'ORDERS':'highPriorityOrder'}, 'complete':{'KITTING':'finished'}})

        with concurent_kitting:
            smach.Concurrence.add('KITTING',  ksm, remapping={'task':'kittingtask'})
            smach.Concurrence.add('ORDERS', CheckOrders(), remapping={'nextOrder':'nextOrder', 'interrupted':'interrupted'})

        smach.StateMachine.add('CKITTING', concurent_kitting, transitions={'complete':'CHECKORDERS', 'interrupted':'HPCHECKTASKS'}, remapping={'task':'task', 'nextOrder':'nextOrder'})

        smach.StateMachine.add('HPCHECKTASKS', CheckTasks(), transitions={'kitting':'KITTING', 'assembly':'ASSEMBLY', 'complete':'CONTINUEINTERRUPTED'}, remapping={'order':'nextOrder', 'task':'HPtask', 'kittingtask':'HPkittingtask'}) #define continue interrupted, save interrupted order
        smach.StateMachine.add('ASSEMBLY', asm, transitions={'finished':'CONTINUEINTERRUPTED'}, remapping={'task':'HPtask', 'kittingtask':'HPkittingtask'}) 
        smach.StateMachine.add('KITTING', ksm, transitions={'finished':'HPCHECKTASKS'}, remapping={'task':'HPkittingtask'})    
        smach.StateMachine.add('CONTINUEINTERRUPTED', ContinueInterrupted(), transitions={'continue':'CHECKORDERS'}, remapping={'interrupted':'interrupted'}) 

    rospy.init_node('main_node', anonymous=True)
    
    outcome = sm.execute()