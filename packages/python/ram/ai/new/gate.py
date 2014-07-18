import ram.ai.new.utilClasses as utilClasses
import ram.ai.new.utilStates as utilStates
import ram.ai.new.motionStates as motion
import ram.ai.new.searchPatterns as searches
import ram.ai.new.stateMachine as stateMachine
import ram.ai.new.state as state
import ram.ai.new.approach as approach

class GateTask(utilStates.Task):
    def __init__(self, pipe, 
                 taskDepth, forwardDistance, searchDistance, 
                 success, failure, duration = 120):
        super(GateTask, self).__init__(GateTaskMachine(pipe,
                                                       taskDepth, 
                                                       forwardDistance,
                                                       searchDistance),
                                       success, failure,
                                       duration)

    def enter(self):
        self.getStateMachine().getLegacyState().visionSystem\
            .pipeLineDetectorOn()
        super(GateTask, self).enter()

    def leave(self):
        self.getStateMachine().getLegacyState().visionSystem\
            .pipeLineDetectorOff()
        super(GateTask, self).leave()

    def update(self):
        if self._InnerMachine().isCompleted() and \
                isinstance(self._InnerMachine().getCurrentState(), 
                           GateFailure):
            self.doTransition(failure)
        super(GateTask, self).update()

    def doFailure(self):
       """
       Method added to initiate failure case from within nested
       machine
       """
       self.doTransition('failure')

class GateTaskMachine(stateMachine.StateMachine):
    def __init__(self, pipe, taskDepth, forwardDistance, searchDistance):
        super(GateTaskMachine, self).__init__()
        
        pipeSearch = searches.ForwardsSearchPattern(
            searchDistance, 
            pipe.isSeen,
            'center', 
            'failure')

        # Add states
        start = self.addState('start', utilStates.Start())
        end = self.addState('end', utilStates.End())
        failure = self.addState('failure', GateFailure())
        dive = self.addState('dive', motion.Dive(taskDepth))
        forward = self.addState('forward', motion.Forward(forwardDistance))
        pipeSearch = self.addState('search', pipeSearch)
        center = self.addState('center', 
                              approach.DownCenter(pipe, 'align', 'failure'))
        align = self.addState('align', 
                              approach.DownOrient(pipe, 'end', 'failure'))

        start.setTransition('next', 'dive')
        dive.setTransition('next', 'forward')
        forward.setTransition('next', 'search')

    def update(self):
        super(GateTaskMachine, self).update()

class GateFailure(utilStates.End):
    def __init__(self):
        super(GateFailure, self).__init__()
    
