#!/usr/bin/env python

import smach

from states import *
from assembly_kitting_states import *
import pick_and_place
from Sensors import Sensors_functions
import path_planning
import process_management
import nist_assembly
import smach_ros

# gets called when ANY child state terminates
def child_term_cb_kitting(outcome_map):

  if outcome_map['ORDERS'] == 'highPriorityOrder':
    return True

  if outcome_map['KITTING']:
    return True

  # in all other case, just keep running, don't terminate anything
  return False


# gets called when ALL child states are terminated
def out_cb_kitting(outcome_map):
    if outcome_map['ORDERS'] == 'highPriorityOrder':
        return 'interrupted'
    elif outcome_map['KITTING'] == 'interrupted':
        return 'interrupted'
    else:
        return 'complete'

# gets called when ANY child state terminates
def child_term_cb_assembly(outcome_map):

    if outcome_map['ORDERS'] == 'highPriorityOrder':
        return True

    if outcome_map['ASSEMBLY']:
        return True

    # in all other case, just keep running, don't terminate anything
    return False


# gets called when ALL child states are terminated
def out_cb_assembly(outcome_map):
    if outcome_map['ORDERS'] == 'highPriorityOrder':
        return 'interrupted'
    elif outcome_map['ASSEMBLY'] == 'interrupted':
        return 'interrupted'
    else:
        return 'complete'


if __name__ == '__main__':

    rospy.init_node('main_node', anonymous=True)
    
    rm = pick_and_place.RobotMover()
    act = rm.inverse_kin
    sen = Sensors_functions()
    gp = path_planning.GantryPlanner()
    node = process_management.process_management() 
    ass = nist_assembly.AssemblyCommander()

    asm = smach.StateMachine(outcomes=['finished', 'interrupted'], input_keys=['task', 'kittingtask'])

    with asm:
        smach.StateMachine.add('CHECKGRIPPER', CheckGripper(act), transitions={'next':'SENDGANTRY', 'changegripper':'CHANGEGRIP', 'preempted':'interrupted'}, remapping={'gripper':'gripper'})
        smach.StateMachine.add('CHANGEGRIP', GetGripper(gp, rm),  transitions={'gripperon':'SENDGANTRY', 'preempted':'interrupted'}, remapping={'gripper':'gripper'})
        smach.StateMachine.add('SENDGANTRY', SendGantry(gp, node, ass), transitions={'arrived':'CHECKPART', 'preempted':'interrupted'}, remapping={'task':'task'})
        smach.StateMachine.add('CHECKPART', CheckPart(node), transitions={'noParts':'SUBMITASSEMBLY', 'newPart':'FINDPART', 'skip':'CHECKPART', 'preempted':'interrupted'}, remapping={'task':'task', 'part':'part'})
        smach.StateMachine.add('SUBMITASSEMBLY', SubmitAssemblyShipment(node), transitions={'success':'finished', 'preempted':'interrupted'}, remapping={'task':'task'})
        smach.StateMachine.add('FINDPART', FindPartOnTray(act, node, sen), transitions={'found':'GANTRYMOVEPART', 'noFound':'CHECKPART', 'preempted':'interrupted'}, remapping={'kittingtask':'kittingtask', 'partcurrentpose': 'partcurrentpose', 'part':'part'})
        smach.StateMachine.add('GANTRYMOVEPART', GantryMovePart(rm, sen, node, ass, gp, act), transitions={'moved':'CHECKPART', 'preempted':'interrupted'}, remapping={'partcurrentpose':'partcurrentpose', 'part':'part', 'task':'task'})

    ksm = smach.StateMachine(outcomes=['finished', 'interrupted'], input_keys=['kittingtask', 'faultybinpose'])
    with ksm:

        # get_gripper_tray_sm = smach.StateMachine(outcomes=['done'], input_keys=['kittingtask', 'trackindex'], output_keys=['gripper', 'traydone'])

        # with get_gripper_tray_sm:
        #     smach.StateMachine.add('CHECKTRAY', CheckMoveableTray(act), transitions={'changegripper':'GETGRIPPER', 'changetray':'GETTRAY'}, remapping={'task':'kittingtask', 'gripper':'gripper'})
        #     smach.StateMachine.add('GETGRIPPER', GetGripper(gp, rm), transitions={'gripperon':'GETTRAY'}, remapping={'gripper':'gripper'})
        #     smach.StateMachine.add('GETTRAY', GantryGetTray(gp, rm, sen), transitions={'trayon':'done'}, remapping={'task':'kittingtask', 'traydone':'traydone'})
                 
        # pick_from_track = smach.StateMachine(outcomes=['finish'], input_keys=['kittingtask', 'traydone'], output_keys=['trackindex'])
        # with pick_from_track:
        #     smach.StateMachine.add('CHECKBELT', WaitConveyorBelt(), transitions={'ontrack':'PICKFROMBELT', 'finish':'finish'}, remapping={'trackindex':'trackindex', 'traydone':'traydone'})
        #     smach.StateMachine.add('PICKFROMBELT', PickFromConveyor(rm, act), transitions={'next':'PICKFROMBELT'}, remapping={'task':'kittingtask', 'trackindex':'trackindex'})
            
        
        # concurrent_tray_track_sm = smach.Concurrence(outcomes=['done'], default_outcome='done', input_keys=['kittingtask', 'task', 'traydone'], output_keys=['gripper'], outcome_map={'done':{'TRAY':'done'}, 'done':{'TRAY':'done', 'TRACK':'finish'}}, child_termination_cb = sm_cb)

        # with concurrent_tray_track_sm:
        #     smach.Concurrence.add('TRACK', pick_from_track, remapping={'task':'kittingtask', 'trackindex':'trackindex', 'traydone':'traydone'})
        #     smach.Concurrence.add('TRAY', get_gripper_tray_sm, remapping={'kittingtask':'kittingtask', 'gripper':'gripper', 'trackindex':'trackindex', 'traydone':'traydone'})

        smach.StateMachine.add('CHECKAGV', CheckAGV(node), transitions={'agvatks':'CHECKTRAY', 'agvnotatks':'MOVEAGVTOKS', 'preempted':'interrupted'}, remapping={'task':'kittingtask'})
        smach.StateMachine.add('MOVEAGVTOKS', SendAGV(node), transitions={'agvatks':'CHECKTRAY', 'preempted':'interrupted'}, remapping={'task':'kittingtask', 'traydone':'traydone'})
        smach.StateMachine.add('CHECKTRAY', CheckMoveableTray(act, rm), transitions={'changegripper':'GETGRIPPER', 'changetray':'GETTRAY', 'preempted':'interrupted'}, remapping={'task':'kittingtask', 'gripper':'gripper'})
        smach.StateMachine.add('GETGRIPPER', GetGripper(gp, rm), transitions={'gripperon':'GETTRAY', 'preempted':'interrupted'}, remapping={'gripper':'gripper'})
        smach.StateMachine.add('GETTRAY', GantryGetTray(node, gp, rm, sen), transitions={'trayon':'CHECKTOREMOVE', 'preempted':'interrupted'}, remapping={'task':'kittingtask', 'traydone':'traydone'})
            
        # smach.StateMachine.add('TRACKANDTRAY', concurrent_tray_track_sm, transitions={'done':'CHECKKITTINGPART'}, remapping={'kittingtask':'kittingtask', 'gripper':'gripper', 'task':'task', 'traydone':'traydone'})
        smach.StateMachine.add('CHECKTOREMOVE', CheckToRemove(node), transitions={'remove':'REMOVE', 'continue':'CHECKKITTINGPART', 'preempted':'interrupted'}, remapping={'positiontoremove':'positiontoremove'})
        smach.StateMachine.add('REMOVE', RemovePart(node, rm, sen), transitions={'removed':'CHECKTOREMOVE', 'preempted':'interrupted'}, remapping={'positiontoremove':'positiontoremove'})
        smach.StateMachine.add('CHECKKITTINGPART', CheckPart(node), transitions={'noParts':'SUBMITKITTING', 'newPart':'FINDPARTINENV', 'skip':'CHECKKITTINGPART', 'preempted':'interrupted'}, remapping={'task':'kittingtask', 'part':'part'})
        smach.StateMachine.add('FINDPARTINENV', FindPartInEnvironment(node, sen), transitions={'found':'KITTINGPICKANDPLACE', 'none':'CHECKKITTINGPART', 'preempted':'interrupted'}, remapping={'task':'kittingtask', 'part':'part', 'partcurrentpose':'partcurrentpose', 'partpose':'partpose'})
        smach.StateMachine.add('KITTINGPICKANDPLACE', KittingRobotPickAndPlace(node, rm, sen), transitions={'success':'CHECKFAULTY', 'lost':'FINDPARTINENV', 'preempted':'interrupted'}, remapping={'task':'kittingtask', 'partpose':'partpose', 'partcurrentpose':'partcurrentpose', 'part':'part'})
        smach.StateMachine.add('CHECKFAULTY', CheckFaulty(), transitions={'faulty':'FAULTYPICKANDPLACE', 'notfaulty':'CHECKKITTINGPART', 'preempted':'interrupted'}, remapping={'part':'part'})
        smach.StateMachine.add('FAULTYPICKANDPLACE', FaultyPickAndPlace(node, rm, sen), transitions={'success':'FINDPARTINENV', 'lost':'FINDPARTINENV', 'preempted':'interrupted'}, remapping={'task':'kittingtask', 'partpose':'faultybinpose', 'partcurrentpose':'partpose', 'part':'part'})
        smach.StateMachine.add('SUBMITKITTING', SubmitKittingShipment(node), transitions={'success':'finished', 'preempted':'interrupted'}, remapping={'task':'kittingtask'})

    hksm = smach.StateMachine(outcomes=['finished', 'interrupted'], input_keys=['kittingtask', 'faultybinpose'])
    with hksm:

        smach.StateMachine.add('CHECKAGV', CheckAGV(node), transitions={'agvatks':'CHECKTRAY', 'agvnotatks':'MOVEAGVTOKS', 'preempted':'interrupted'}, remapping={'task':'kittingtask'})
        smach.StateMachine.add('MOVEAGVTOKS', SendAGV(node), transitions={'agvatks':'CHECKTRAY', 'preempted':'interrupted'}, remapping={'task':'kittingtask', 'traydone':'traydone'})
        smach.StateMachine.add('CHECKTRAY', CheckMoveableTray(act, rm), transitions={'changegripper':'GETGRIPPER', 'changetray':'GETTRAY', 'preempted':'interrupted'}, remapping={'task':'kittingtask', 'gripper':'gripper'})
        smach.StateMachine.add('GETGRIPPER', GetGripper(gp, rm), transitions={'gripperon':'GETTRAY', 'preempted':'interrupted'}, remapping={'gripper':'gripper'})
        smach.StateMachine.add('GETTRAY', GantryGetTray(node, gp, rm, sen), transitions={'trayon':'CHECKTOREMOVE', 'preempted':'interrupted'}, remapping={'task':'kittingtask', 'traydone':'traydone'})
            
        # smach.StateMachine.add('TRACKANDTRAY', concurrent_tray_track_sm, transitions={'done':'CHECKKITTINGPART'}, remapping={'kittingtask':'kittingtask', 'gripper':'gripper', 'task':'task', 'traydone':'traydone'})
        smach.StateMachine.add('CHECKTOREMOVE', CheckToRemove(node), transitions={'remove':'REMOVE', 'continue':'HCHECKKITTINGPART', 'preempted':'interrupted'}, remapping={'positiontoremove':'positiontoremove'})
        smach.StateMachine.add('REMOVE', RemovePart(node, rm, sen), transitions={'removed':'CHECKTOREMOVE', 'preempted':'interrupted'}, remapping={'positiontoremove':'positiontoremove'})
        smach.StateMachine.add('HCHECKKITTINGPART', CheckPart(node), transitions={'noParts':'SUBMITKITTING', 'newPart':'FINDPARTINENV', 'skip':'HCHECKKITTINGPART', 'preempted':'interrupted'}, remapping={'task':'kittingtask', 'part':'part'})
        smach.StateMachine.add('FINDPARTINENV', FindPartInEnvironment(node, sen), transitions={'found':'KITTINGPICKANDPLACE', 'none':'HCHECKKITTINGPART', 'preempted':'interrupted'}, remapping={'task':'kittingtask', 'part':'part', 'partcurrentpose':'partcurrentpose', 'partpose':'partpose'})
        smach.StateMachine.add('KITTINGPICKANDPLACE', KittingRobotPickAndPlace(node, rm, sen), transitions={'success':'CHECKFAULTY', 'lost':'FINDPARTINENV', 'preempted':'interrupted'}, remapping={'task':'kittingtask', 'partpose':'partpose', 'partcurrentpose':'partcurrentpose', 'part':'part'})
        smach.StateMachine.add('CHECKFAULTY', CheckFaulty(), transitions={'faulty':'FAULTYPICKANDPLACE', 'notfaulty':'HCHECKKITTINGPART', 'preempted':'interrupted'}, remapping={'part':'part'})
        smach.StateMachine.add('FAULTYPICKANDPLACE', FaultyPickAndPlace(node, rm, sen), transitions={'success':'FINDPARTINENV', 'lost':'FINDPARTINENV', 'preempted':'interrupted'}, remapping={'task':'kittingtask', 'partpose':'faultybinpose', 'partcurrentpose':'partpose', 'part':'part'})
        smach.StateMachine.add('SUBMITKITTING', SubmitKittingShipment(node), transitions={'success':'finished', 'preempted':'interrupted'}, remapping={'task':'kittingtask'})

    simple_kitting = smach.StateMachine(outcomes=['finished', 'interrupted'], input_keys=['kittingtask', 'faultybinpose'])
    with simple_kitting:

        # smach.StateMachine.add('TRACKANDTRAY', concurrent_tray_track_sm, transitions={'done':'CHECKKITTINGPART'}, remapping={'kittingtask':'kittingtask', 'gripper':'gripper', 'task':'task', 'traydone':'traydone'})
        smach.StateMachine.add('CHECKTOREMOVE', CheckToRemove(node), transitions={'remove':'REMOVE', 'continue':'CHECKKITTINGPART', 'preempted':'interrupted'}, remapping={'positiontoremove':'positiontoremove'})
        smach.StateMachine.add('REMOVE', RemovePart(node, rm, sen), transitions={'removed':'CHECKTOREMOVE', 'preempted':'interrupted'}, remapping={'positiontoremove':'positiontoremove'})
        smach.StateMachine.add('CHECKKITTINGPART', CheckPart(node), transitions={'noParts':'SUBMITKITTING', 'newPart':'FINDPARTINENV', 'skip':'CHECKKITTINGPART', 'preempted':'interrupted'}, remapping={'task':'kittingtask', 'part':'part'})
        smach.StateMachine.add('FINDPARTINENV', FindPartInEnvironment(node, sen), transitions={'found':'KITTINGPICKANDPLACE', 'none':'CHECKKITTINGPART', 'preempted':'interrupted'}, remapping={'task':'kittingtask', 'part':'part', 'partcurrentpose':'partcurrentpose', 'partpose':'partpose'})
        smach.StateMachine.add('KITTINGPICKANDPLACE', KittingRobotPickAndPlace(node, rm, sen), transitions={'success':'CHECKFAULTY', 'lost':'FINDPARTINENV', 'preempted':'interrupted'}, remapping={'task':'kittingtask', 'partpose':'partpose', 'partcurrentpose':'partcurrentpose', 'part':'part'})
        smach.StateMachine.add('CHECKFAULTY', CheckFaulty(), transitions={'faulty':'FAULTYPICKANDPLACE', 'notfaulty':'CHECKKITTINGPART', 'preempted':'interrupted'}, remapping={'part':'part'})
        smach.StateMachine.add('FAULTYPICKANDPLACE', FaultyPickAndPlace(node, rm, sen), transitions={'success':'FINDPARTINENV', 'lost':'FINDPARTINENV', 'preempted':'interrupted'}, remapping={'task':'kittingtask', 'partpose':'faultybinpose', 'partcurrentpose':'partpose', 'part':'part'})
        smach.StateMachine.add('SUBMITKITTING', SubmitKittingShipment(node), transitions={'success':'finished', 'preempted':'interrupted'}, remapping={'task':'kittingtask'})

     
    sm = smach.StateMachine(outcomes=['end'], input_keys=['faultybinpose'])

    with sm:

        smach.StateMachine.add('START', StartCompetition(node), transitions={'success':'CHECKORDERS'}, remapping={'interrupted':'interrupted'})
        smach.StateMachine.add('CHECKORDERS', CheckOrders(node), transitions={'complete':'ENDCOMP', 'nextOrder':'CHECKTASKS', 'highPriorityOrder':'HPCHECKTASKS'}, remapping={'nextOrder':'order'})
        smach.StateMachine.add('CHECKTASKS', CheckTasks(node), transitions={'kitting':'CKITTING', 'assembly':'CASSEMBLY', 'complete':'CHECKORDERS', 'kittingnotray':'SIMPLEKITTING'}, remapping={'order':'order', 'task':'task'})
        smach.StateMachine.add('ENDCOMP', EndCompetition(node), transitions={'ended':'end'})

        # waitorder = smach.StateMachine(outcomes=['highPriorityOrder'], output_keys=['nextOrder'])
        # with waitorder:
        #     smach.StateMachine.add('WAITFORORDER', CheckOrders(node), transitions={'complete':'WAITFORORDER', 'highPriorityOrder':'highPriorityOrder', 'nextOrder':'WAITFORORDER'}, remapping={'nextOrder':'nextOrder'})
   
        concurrent_assembly = smach.Concurrence(outcomes=['interrupted', 'complete'], default_outcome='complete', input_keys=['task', 'kittingtask', 'interrupted'], output_keys=['nextOrder'], child_termination_cb=child_term_cb_assembly, outcome_cb=out_cb_assembly)

        with concurrent_assembly:
            smach.Concurrence.add('ASSEMBLY',  asm, remapping={'task':'task', 'kittingtask':'kittingtask'})
            smach.Concurrence.add('ORDERS', WaitOrder(node), remapping={'nextOrder':'nextOrder'})
        
        smach.StateMachine.add('CASSEMBLY', concurrent_assembly, transitions={'complete':'CHECKORDERS', 'interrupted':'STOREASSEMBLYTASK'}, remapping={'task':'task', 'kittingtask':'kittingtask','nextOrder':'nextOrder', 'interrupted':'interrupted'})
        smach.StateMachine.add('STOREASSEMBLYTASK', StoreTask(node), transitions={'stored':'HPCHECKTASKS'}, remapping={'task':'task'})
        #concurrent_kitting = smach.Concurrence(outcomes=['interrupted', 'complete'], default_outcome='complete', input_keys=['kittingtask', 'interrupted', 'faultybinpose'], output_keys=['nextOrder'], outcome_map={'interrupted':{'ORDERS':'highPriorityOrder'}, 'complete':{'KITTING':'finished'}})
        concurrent_kitting = smach.Concurrence(outcomes=['interrupted', 'complete'], default_outcome='complete', input_keys=['kittingtask', 'interrupted', 'faultybinpose'], output_keys=['nextOrder'], child_termination_cb=child_term_cb_kitting, outcome_cb=out_cb_kitting)

        with concurrent_kitting:
            smach.Concurrence.add('KITTING',  ksm, remapping={'kittingtask':'kittingtask', 'faultybinpose':'faultybinpose'})
            smach.Concurrence.add('ORDERS', WaitOrder(node), remapping={'nextOrder':'nextOrder'})

        smach.StateMachine.add('SIMPLEKITTING', simple_kitting, transitions={'finished':'CHECKTASKS', 'interrupted':'CHECKTASKS'})
        smach.StateMachine.add('CKITTING', concurrent_kitting, transitions={'complete':'CHECKTASKS', 'interrupted':'STOREKITTINGTASK'}, remapping={'kittigtask':'kittingtask', 'nextOrder':'nextOrder', 'interrupted':'interrupted', 'faultybinpose':'faultybinpose'})
        smach.StateMachine.add('STOREKITTINGTASK', StoreTask(node), transitions={'stored':'HPCHECKTASKS'}, remapping={'task':'kittingtask'})
        smach.StateMachine.add('HPCHECKTASKS', HPCheckTasks(node), transitions={'kitting':'HKITTING', 'assembly':'HASSEMBLY', 'complete':'CONTINUEINTERRUPTED'}, remapping={'order':'nextOrder', 'HPtask':'HPtask', 'HPkittingtask':'HPkittingtask'}) #define continue interrupted, save interrupted order
        smach.StateMachine.add('HASSEMBLY', asm, transitions={'finished':'CONTINUEINTERRUPTED', 'interrupted':'end'}, remapping={'task':'HPtask', 'kittingtask':'HPkittingtask'}) 
        smach.StateMachine.add('HKITTING', hksm, transitions={'finished':'HPCHECKTASKS', 'interrupted':'end'}, remapping={'kittingtask':'HPkittingtask', 'faultybinpose':'faultybinpose'})    
        smach.StateMachine.add('CONTINUEINTERRUPTED', ContinueInterrupted(), transitions={'continue':'CHECKORDERS'}, remapping={'interrupted':'interrupted'}) 

    faultybin = Pose()
    faultybin.position.x = -2.186829
    faultybin.position.y = 0.1
    faultybin.position.z = 0.8

    sm.userdata.faultybinpose = faultybin

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()


    outcome = sm.execute()
