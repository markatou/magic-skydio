package lggltl.exp;

import burlap.behavior.policy.Policy;
import burlap.behavior.policy.PolicyUtils;
import burlap.behavior.singleagent.Episode;
import burlap.behavior.singleagent.auxiliary.EpisodeSequenceVisualizer;
import burlap.behavior.singleagent.planning.Planner;
import burlap.behavior.singleagent.planning.stochastic.rtdp.BoundedRTDP;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.behavior.valuefunction.ConstantValueFunction;
import burlap.mdp.core.Domain;
import burlap.mdp.core.TerminalFunction;
import burlap.mdp.core.oo.propositional.GroundedProp;
import burlap.mdp.core.oo.propositional.PropositionalFunction;
import burlap.mdp.core.oo.state.OOState;
import burlap.mdp.core.state.State;
import burlap.mdp.singleagent.SADomain;
import burlap.mdp.singleagent.environment.Environment;
import burlap.mdp.singleagent.environment.SimulatedEnvironment;
import burlap.mdp.singleagent.model.FactoredModel;
import burlap.mdp.singleagent.model.RewardFunction;
import burlap.mdp.singleagent.oo.OOSADomain;
import burlap.statehashing.HashableStateFactory;
import burlap.statehashing.simple.SimpleHashableStateFactory;
import burlap.visualizer.Visualizer;
import amdp.cleanup.CleanupDomain;
import amdp.cleanup.CleanupVisualiser;
import lggltl.cleanup.CreateStartStates;
import amdp.cleanup.state.CleanupState;
import lggltl.gltl.GLTLCompiler;
import lggltl.gltl.state.GLTLState;
import org.yaml.snakeyaml.Yaml;

import java.text.Normalizer;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

/**
 * Created by dilip on 10/26/17.
 */
public class CleanupGLTL {

    public static void main(String[] args) {

        String formula;

//        formula = Formulas.ALW_EVENT_BLUE_AND_EVENT_YELLOW;
//        formula = Formulas.ROTATE_FOUR_ROOMS;
//        formula = Formulas.EVENT_BLOCK2GREEN_AND_NEVER_BLUE;
//        formula = Formulas.EVENT_BLOCK2GREEN_AND_NEVER_YELLOW;
//        formula = Formulas.EVENT_BLUE_NEVER_GREEN;
//        formula = "F4&BF4C";

        formula = Formulas.EVENT_BLOCK2GREEN_AND_NEVER_BLUE_UNTIL_BLOCK2GREEN;

        if (args.length > 0) {
            formula = args[0];
        }

        System.out.println("Running for formula " + formula);

        final OOSADomain envDomain;

        CleanupDomain dgen = new CleanupDomain();
        dgen.includeDirectionAttribute(true);
        dgen.includePullAction(true);
        dgen.includeWallPF_s(true);
        dgen.includeLockableDoors(false);
        dgen.setLockProbability(0.0);
        envDomain = dgen.generateDomain();

//        State s = CleanupDomain.getClassicState(true);
//        State s = CleanupDomain.getState(true, false, 1, 4);
        State s = new CreateStartStates().getState("2_pink_yellow_green_blue");



//        Map<String, GroundedProp> symbolMap = SymbolMaps.CS_SYMBOL_MAP;
        Map<String, GroundedProp> symbolMap = SymbolMaps.getSymbolMap((CleanupState) s);

        GLTLCompiler compiler = new GLTLCompiler(formula, symbolMap, envDomain);
        Domain compiledDomain = compiler.generateDomain();

        State initialCompiledState = compiler.addInitialTaskStateToEnvironmentState((OOState) s);
        // System.out.println(initialCompiledState.getCompleteStateDescription());

        HashableStateFactory hashingFactory = new SimpleHashableStateFactory();

        //begin planning in our compiled domain
//        Planner planner = new ValueIteration((SADomain) compiledDomain, 1.0, hashingFactory, 0.0000001, 1000);
//        ((ValueIteration) planner).toggleReachabiltiyTerminalStatePruning(true);

        Planner planner = new BoundedRTDP((SADomain) compiledDomain, 1.0, hashingFactory, new ConstantValueFunction(0.0), new ConstantValueFunction(1.0),0.0000001, 1000);
        long startTime = System.nanoTime();
        Policy p = planner.planFromState(initialCompiledState);
        long endTime = System.nanoTime();

        long duration = (endTime - startTime) / 1000000000;
        System.out.println("BRTDP took " + duration + " seconds");

        Environment env = new SimulatedEnvironment((SADomain)compiledDomain,initialCompiledState);

        Episode ea = PolicyUtils.rollout(p, env);

        Episode cw_episode= new Episode(((GLTLState)ea.stateSequence.get(0)).envState);

        for(int count = 0;count<ea.numActions();count++){
            CleanupState gsNew = (CleanupState) ((GLTLState)ea.stateSequence.get(count+1)).envState;
            cw_episode.transition(ea.actionSequence.get(count),gsNew,0);
        }

        Visualizer v = CleanupVisualiser.getVisualizer("data/resources/robotImages");
        new EpisodeSequenceVisualizer(v, envDomain, Arrays.asList(cw_episode));
    }
}
