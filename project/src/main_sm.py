#!/usr/bin/env python

import smach

from states import *
from assembly_kitting_states import *
import pick_and_place
from Sensors import Sensors_functions
import path_planning
import process_management

if __name__ == '__main__':

    rm = pick_and_place.RobotMover()
    act = rm.inverse_kin
    sen = Sensors_functions()
    gp = path_planning.GantryPlanner()
    node = process_management.process_management()


    asm = smach.StateMachine(outcomes=['finished'], input_keys=['task', 'kittingtask'])

    with asm:
        smach.StateMachine.add('CHECKGRIPPER', CheckGripper(act), transitions={'next':'SENDGANTRY', 'changegripper':'CHANGEGRIP'}, remapping={'gripper':'gripper'})
        smach.StateMachine.add('CHANGEGRIP', GetGripper(gp, rm),  transitions={'gripperon':'SENDGANTRY'}, remapping={'gripper':'gripper'})
        smach.StateMachine.add('SENDGANTRY', SendGantry(gp), transitions={'arrived':'CHECKPART'}, remapping={'task':'task'})
        smach.StateMachine.add('CHECKPART', CheckPart(), transitions={'noParts':'SUBMITASSEMBLY', 'newPart':'FINDPART'}, remapping={'task':'task', 'part':'part'})
        smach.StateMachine.add('SUBMITASSEMBLY', SubmitAssemblyShipment(node), transitions={'success':'finished'}, remapping={'task':'task'})
        smach.StateMachine.add('FINDPART', FindPartOnTray(), transitions={'found':'GANTRYMOVEPART'}, remapping={'kittingtask':'kittingtask', 'partcurrentposition': 'partcurrentposition', 'part':'part'})
        smach.StateMachine.add('GANTRYMOVEPART', GantryMovePart(rm, sen), transitions={'moved':'CHECKPART'}, remapping={'partcurrentposition':'partcurrentposition', 'part':'part'})

    ksm = smach.StateMachine(outcomes=['finished'], input_keys=['kittingtask', 'faultybinposition'])
    with ksm:
        smach.StateMachine.add('CHECKAGV', CheckAGV(node), transitions={'agvatks':'CHECKTRAY', 'agvnotatks':'MOVEAGVTOKS'}, remapping={'task':'kittingtask'})
        smach.StateMachine.add('MOVEAGVTOKS', SendAGV(node), transitions={'agvatks':'CHECKTRAY'}, remapping={'task':'kittingtask'})
        smach.StateMachine.add('CHECKTRAY', CheckMoveableTray(act), transitions={'changegripper':'GETGRIPPER', 'changetray':'GETTRAY'}, remapping={'task':'kittingtask', 'gripper':'gripper'})
        smach.StateMachine.add('GETGRIPPER', GetGripper(gp, rm), transitions={'gripperon':'GETTRAY'}, remapping={'gripper':'gripper'})
        smach.StateMachine.add('GETTRAY', GantryGetTray(gp, rm, sen), transitions={'trayon':'CHECKKITTINGPART'}, remapping={'task':'kittingtask'})
        smach.StateMachine.add('CHECKKITTINGPART', CheckPart(), transitions={'noParts':'SUBMITKITTING', 'newPart':'FINDPARTINENV'}, remapping={'task':'kittingtask', 'part':'part'})
        smach.StateMachine.add('FINDPARTINENV', FindPartInEnvironment(sen), transitions={'found':'KITTINGPICKANDPLACE'}, remapping={'part':'part', 'partcurrentposition':'partcurrentposition', 'partposition':'partposition'})
        smach.StateMachine.add('KITTINGPICKANDPLACE', KittingRobotPickAndPlace(rm), transitions={'success':'CHECKFAULTY'}, remapping={'partposition':'partposition', 'partcurrentposition':'partcurrentposition'})
        smach.StateMachine.add('CHECKFAULTY', CheckFaulty(), transitions={'faulty':'FAULTYPICKANDPLACE', 'notfaulty':'CHECKKITTINGPART'}, remapping={'part':'part'})
        smach.StateMachine.add('FAULTYPICKANDPLACE', KittingRobotPickAndPlace(rm), transitions={'success':'FINDPARTINENV'}, remapping={'partposition':'faultybinposition', 'partcurrentposition':'partposition'})
        smach.StateMachine.add('SUBMITKITTING', SubmitKittingShipment(node), transitions={'success':'finished'}, remapping={'task':'kittingtask'})

    sm = smach.StateMachine(outcomes=['end'], input_keys=['faultybinposition'])

    with sm:

        smach.StateMachine.add('START', StartCompetition(node), transitions={'success':'CHECKORDERS'}, remapping={'interrupted':'interrupted'})
        smach.StateMachine.add('CHECKORDERS', CheckOrders(node), transitions={'complete':'end', 'nextOrder':'CHECKTASKS', 'highPriorityOrder':'HPCHECKTASKS'}, remapping={'nextOrder':'order', 'interrupted':'interrupted'})
        smach.StateMachine.add('CHECKTASKS', CheckTasks(), transitions={'kitting':'CKITTING', 'assembly':'CASSEMBLY', 'complete':'CHECKORDERS'}, remapping={'order':'order', 'task':'task'})

        concurent_assembly = smach.Concurrence(outcomes=['interrupted', 'complete'], default_outcome='complete', input_keys=['task'], output_keys=['nextOrder'], outcome_map={'interrupted':{'ORDERS':'highPriorityOrder'}, 'complete':{'ASSEMBLY':'finished'}})

        with concurent_assembly:
            smach.Concurrence.add('ASSEMBLY',  asm, remapping={'task':'task', 'kittingtask':'kittingtask'})
            smach.Concurrence.add('ORDERS', CheckOrders(node), remapping={'nextOrder':'nextOrder', 'interrupted':'interrupted'})
        
        smach.StateMachine.add('CASSEMBLY', concurent_assembly, transitions={'complete':'CHECKORDERS', 'interrupted':'HPCHECKTASKS'}, remapping={'task':'task', 'nextOrder':'nextOrder'})
        
        concurent_kitting = smach.Concurrence(outcomes=['interrupted', 'complete'], default_outcome='complete', input_keys=['kittingtask', 'interrupted', 'faultybinposition'], output_keys=['nextOrder'], outcome_map={'interrupted':{'ORDERS':'highPriorityOrder'}, 'complete':{'KITTING':'finished'}})

        with concurent_kitting:
            smach.Concurrence.add('KITTING',  ksm, remapping={'task':'kittingtask', 'faultybinposition':'faultybinposition'})
            smach.Concurrence.add('ORDERS', CheckOrders(node), remapping={'nextOrder':'nextOrder', 'interrupted':'interrupted'})

        smach.StateMachine.add('CKITTING', concurent_kitting, transitions={'complete':'CHECKORDERS', 'interrupted':'HPCHECKTASKS'}, remapping={'kittigtask':'kittingtask', 'nextOrder':'nextOrder', 'interrupted':'interrupted', 'faultybinposition':'faultybinposition'})

        smach.StateMachine.add('HPCHECKTASKS', CheckTasks(), transitions={'kitting':'KITTING', 'assembly':'ASSEMBLY', 'complete':'CONTINUEINTERRUPTED'}, remapping={'order':'nextOrder', 'task':'HPtask', 'kittingtask':'HPkittingtask'}) #define continue interrupted, save interrupted order
        smach.StateMachine.add('ASSEMBLY', asm, transitions={'finished':'CONTINUEINTERRUPTED'}, remapping={'task':'HPtask', 'kittingtask':'HPkittingtask'}) 
        smach.StateMachine.add('KITTING', ksm, transitions={'finished':'HPCHECKTASKS'}, remapping={'task':'HPkittingtask'})    
        smach.StateMachine.add('CONTINUEINTERRUPTED', ContinueInterrupted(), transitions={'continue':'CHECKORDERS'}, remapping={'interrupted':'interrupted'}) 

    faultybin = Pose()
    faultybin.position.x = 0
    faultybin.position.y = 0
    faultybin.position.z = 0

    sm.userdata.faultybinposition = faultybin.position

    rospy.init_node('main_node', anonymous=True)
    
    outcome = sm.execute()