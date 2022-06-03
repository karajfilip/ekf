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
    def __init__(self, processmgmt, outcomes=['complete', 'nextOrder', 'highPriorityOrder'], output_keys=['nextOrder']):
        smach.State.__init__(self, outcomes, output_keys=output_keys)
        self.node = processmgmt
        self.node.orders_served = 0
        self.lastorder = None

    def execute(self, ud):
        while not self.node.received_order:
            pass
        
        rospy.logerr(self.node.orders)
        if len(self.node.orders)==0:
            rospy.sleep(5)
        if len(self.node.orders)==0:
            return 'complete'
        else:
            order = self.node.orders[-1]
            ud.nextOrder = order
            if order.order_id == 'order_1' or 'update' in order.order_id:
                self.node.orders_served += 1
                if 'update' in order.order_id:
                    self.node.orders.remove(self.node.orders[-2])
                return 'highPriorityOrder'
            return 'nextOrder'

class WaitOrder(smach.State):
    def __init__(self, processmgmt, outcomes=['complete', 'nextOrder', 'highPriorityOrder'], output_keys=['nextOrder']):
        smach.State.__init__(self, outcomes, output_keys=output_keys)
        self.node = processmgmt

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            rospy.logwarn('PREEMPTED')
            return 'complete'
        
        self.n = len(self.node.orders)
        while len(self.node.orders)<=self.n:
            pass

        rospy.logerr(self.node.orders)
        if len(self.node.orders)>0:

            order = self.node.orders[-1]
            ud.nextOrder = order
            if order.order_id == 'order_1' or 'update' in order.order_id:
                self.node.orders_served += 1
                if 'update' in order.order_id:
                    self.node.orders.remove(self.node.orders[-2])
                return 'highPriorityOrder'
            return 'nextOrder'
        return 'complete'


class CheckTasks(smach.State):
    def __init__(self, processmgmt, outcomes=['kitting', 'assembly', 'complete', 'kittingnotray'], input_keys=['order'], output_keys=['task', 'kittingtask']):
        smach.State.__init__(self, outcomes, input_keys, output_keys)
        self.node = processmgmt
        self.order = Order()
        self.ksi = 0
        self.asi = 0

    def execute(self, ud):
        rospy.logwarn('TASKS')
        if self.node.interrupted:
            if 'kitting' in str(self.node.interrupted.shipment_type):
                task = self.node.interrupted
                ud.kittingtask = task
                rospy.logerr(task.agv)
                rospy.logwarn(self.node.placed)
                if task.agv in self.node.placed:
                    for product in task.products:
                        self.node.remove.extend([p for t,p,rp in self.node.placed[task.agv] if t == product.type and rp != product.pose])
                        self.node.skip.extend([t for t,p,rp in self.node.placed[task.agv] if t == product.type and rp == product.pose])
                else:
                    self.node.placed[task.agv] = list()
                self.node.interrupted = None
                return 'kittingnotray'
            elif 'assembly' in str(self.node.interrupted.shipment_type):
                ud.task = self.node.interrupted
                self.node.interrupted = None
                return 'assemblynotray'
        if ud.order != self.order:
            self.ksi = 0
            self.asi = 0
            ud.kittingtask = None
            self.order = ud.order
            self.node.skip = list()
            self.node.remove = list()
        if self.ksi < len(ud.order.kitting_shipments):            
            task = ud.order.kitting_shipments[self.ksi]
            ud.kittingtask = task
            self.ksi += 1
            if task.assembly_station in self.node.placed:
                for product in task.products:
                    self.node.remove.extend([p for t,p,rp in self.node.placed[task.assembly_station] if t == product.type and rp != product.pose])
                    self.node.skip.extend([t for t,p,rp in self.node.placed[task.assembly_station] if t == product.type and rp == product.pose])
            return 'kitting'
        elif self.asi < len(ud.order.assembly_shipments):
            ud.task = ud.order.assembly_shipments[self.asi]
            self.asi += 1
            return 'assembly'
        else:
            self.node.orders.remove(ud.order)
            return 'complete'

class HPCheckTasks(smach.State):
    def __init__(self, processmgmt, outcomes=['kitting', 'assembly', 'complete'], input_keys=['order'], output_keys=['HPtask', 'HPkittingtask']):
        smach.State.__init__(self, outcomes, input_keys, output_keys)
        self.node = processmgmt
        self.order = Order()
        self.ksi = 0
        self.asi = 0

    def execute(self, ud):
        if ud.order != self.order:
            self.ksi = 0
            self.asi = 0
            ud.HPkittingtask = None
            self.node.remove = list()
            self.node.skip = list()
            self.order = ud.order
        if self.ksi < len(ud.order.kitting_shipments):
            task = ud.order.kitting_shipments[self.ksi]
            ud.HPkittingtask = task
            self.ksi += 1
            if task.assembly_station in self.node.placed:
                for product in task.products:
                    self.node.remove.extend([p for t,p,rp in self.node.placed[task.assembly_station] if t == product.type and rp != product.pose])
                    self.node.skip.extend([t for t,p,rp in self.node.placed[task.assembly_station] if t == product.type and rp == product.pose])
            return 'kitting'
        elif self.asi < len(ud.order.assembly_shipments):
            ud.HPtask = ud.order.assembly_shipments[self.asi]
            self.asi += 1
            return 'assembly'
        else:
            self.node.orders.remove(ud.order)
            return 'complete'

class ContinueInterrupted(smach.State):
    def __init__(self, outcomes=['continue'], output_keys=['interrupted']):
        smach.State.__init__(self, outcomes, output_keys)

    def execute(self, ud):
        return 'continue'

class StoreTask(smach.State):
    def __init__(self, processmgmt, outcomes=['stored'], input_keys=['task']):
        smach.State.__init__(self, outcomes, input_keys)
        self.node = processmgmt

    def execute(self, ud):
        self.node.interrupted = ud.task
        rospy.logerr('INERRUPTED')
        rospy.logerr(self.node.interrupted.shipment_type)
        return 'stored'


