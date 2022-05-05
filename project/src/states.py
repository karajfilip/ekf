import rospy
import smach
import process_management

from nist_gear.msg import Order

class StartCompetition(smach.State):
    def __init__(self, processmgmt, outcomes=['success'], output_keys=['interrupted']):
        smach.State.__init__(self, outcomes, output_keys=output_keys)
        self.node = processmgmt

    def execute(self, ud):
        ud.interrupted = False
        self.node.start_competition()
        return 'success'

class EndCompetition(smach.State):
    def __init__(self, processmgmt, outcomes=['ended']):
        smach.State.__init__(self, outcomes)
        self.node = processmgmt
    
    def execute(self, ud):
        self.node.end_competition()
        return 'ended'

class CheckOrders(smach.State):
    def __init__(self, processmgmt, outcomes=['complete', 'nextOrder', 'highPriorityOrder'], input_keys=['interrupted'], output_keys=['nextOrder']):
        smach.State.__init__(self, outcomes, output_keys=output_keys, input_keys=input_keys)
        self.node = processmgmt
        self.i = 0

    def execute(self, ud):
        while not self.node.received_order:
            rospy.loginfo("Waiting for order...")
            pass

        if ud.interrupted == 'true':
            self.i -= 1
                
        if len(self.node.orders)==self.i:
            return 'complete'
        else:
            order = self.node.orders[self.i]
            self.i += 1
            ud.nextOrder = order
            if hasattr(order, 'priority'):
                if order.priority == 3:
                    return 'highPriorityOrder'
            return 'nextOrder'

class CheckTasks(smach.State):
    def __init__(self, outcomes=['kitting', 'assembly', 'complete'], input_keys=['order'], output_keys=['task', 'kittingtask']):
        smach.State.__init__(self, outcomes, input_keys, output_keys)
        self.order = Order()
        self.ksi = 0
        self.asi = 0

    def execute(self, ud):
        if ud.order != self.order:
            self.ksi = 0
            self.asi = 0
            ud.kittingtask = None
            self.order = ud.order
        if self.ksi < len(ud.order.kitting_shipments):
            ud.kittingtask = ud.order.kitting_shipments[self.ksi]
            self.ksi += 1
            return 'kitting'
        elif self.asi < len(ud.order.assembly_shipments):
            ud.task = ud.order.assembly_shipments[self.asi]
            self.asi += 1
            return 'assembly'
        else:
            return 'complete'

class ContinueInterrupted(smach.State):
    def __init__(self, outcomes=['continue'], output_keys=['interrupted']):
        smach.State.__init__(self, outcomes, output_keys)

    def execute(self, ud):
        ud.interrupted = True
        return 'continue'

