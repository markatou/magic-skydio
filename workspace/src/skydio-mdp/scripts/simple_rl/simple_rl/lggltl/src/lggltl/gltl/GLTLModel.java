package lggltl.gltl;

import burlap.mdp.core.StateTransitionProb;
import burlap.mdp.core.action.Action;
import burlap.mdp.core.action.ActionType;
import burlap.mdp.core.oo.state.OOState;
import burlap.mdp.core.state.State;
import burlap.mdp.singleagent.model.statemodel.FullStateModel;
import lggltl.gltl.state.GLTLState;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by ngopalan on 10/24/17.
 */
public class GLTLModel implements FullStateModel {
    FullStateModel sourceModel;

    public GLTLModel(FullStateModel model){
        sourceModel = model;
    }

    @Override
    public List<StateTransitionProb> stateTransitions(State state, Action action) {

//        System.out.println("Querying state transitions for " + state + "\t" + action);

        CompiledAction ca = (CompiledAction)action;
        //get the environment mdp transition dynamics
        List<StateTransitionProb> environmentTPs = this.sourceModel.stateTransitions(((GLTLState)state).envState, ca.getSourceAction());

        //reserve space for the joint task-environment mdp transitions
        List<StateTransitionProb> jointTPs = new ArrayList<StateTransitionProb>(environmentTPs.size() * 2);

        //perform outer loop of transitions cross product over environment transitions
        for (StateTransitionProb etp : environmentTPs) {

            //get the task transitions and expand them with the environment transition
            List<GLTLCompiler.TaskMDPTransition> taskTPs = ca.getCompiledActionType().getTaskTransitions(state, etp.s);
//				System.out.println("===>" + taskTPs.size());
            double taskSum = 0.;
            for (GLTLCompiler.TaskMDPTransition ttp : taskTPs) {
                GLTLState ns = new GLTLState((OOState)etp.s.copy(), ttp.task);

                //remove the old task spec
//                ns.removeObject(ns.getFirstObjectOfClass(CLASSSPEC).getName());
                //set the new task spec
//                ns.spec = ttp.task;
//                ns.addObject(ttp.taskObject);
                double p = etp.p * ttp.p;
                StateTransitionProb jtp = new StateTransitionProb(ns, p);
                jointTPs.add(jtp);

                taskSum += ttp.p;
            }

            if (Math.abs(taskSum - 1.) > 1e-5) {
                throw new RuntimeException("Error, could not return transition probabilities because task MDP transition probabilities summed to " + taskSum + " instead of 1.");
            }

        }

        return jointTPs;

    }

    @Override
    public State sample(State state, Action action) {
        return FullStateModel.Helper.sampleByEnumeration(this, state, action);
    }
}
