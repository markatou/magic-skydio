package lggltl.exp;

import burlap.behavior.policy.GreedyQPolicy;
import burlap.behavior.policy.Policy;
import burlap.behavior.policy.PolicyUtils;
import burlap.behavior.singleagent.Episode;
import burlap.behavior.singleagent.auxiliary.EpisodeSequenceVisualizer;
import burlap.behavior.singleagent.auxiliary.StateEnumerator;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.domain.singleagent.gridworld.GridWorldVisualizer;
import burlap.domain.singleagent.gridworld.state.GridAgent;
import burlap.domain.singleagent.gridworld.state.GridLocation;
import burlap.domain.singleagent.gridworld.state.GridWorldState;
import burlap.mdp.core.Domain;
import burlap.mdp.core.TerminalFunction;
import burlap.mdp.core.action.Action;
import burlap.mdp.core.action.ActionType;
import burlap.mdp.core.oo.propositional.GroundedProp;
import burlap.mdp.core.oo.propositional.PropositionalFunction;
import burlap.mdp.core.oo.state.OOState;
import burlap.mdp.core.state.State;
import burlap.mdp.singleagent.environment.Environment;
import burlap.mdp.singleagent.environment.EnvironmentOutcome;
import burlap.mdp.singleagent.environment.SimulatedEnvironment;
import burlap.mdp.singleagent.model.RewardFunction;
import burlap.mdp.singleagent.oo.OOSADomain;
import burlap.statehashing.simple.SimpleHashableStateFactory;
import burlap.visualizer.Visualizer;
import lggltl.gltl.GLTLCompiler;
import lggltl.gltl.state.GLTLState;

import java.util.*;
import java.awt.Graphics2D;

/**
 * @author James MacGlashan. ... tweaker Michael Littman ... ported to BURLAP 3 by ngopalan
 */
public class Trace {

    public static final String PFINORANGE = "InOrange";
    public static final String PFINBLUE = "InBlue";
    public static final String PFNOTINBLUE = "NotInBlue";
    public static final String LOCATIONPREFIX = "location";

    public static void main(String[] args) {

        String formula;
        char grid;


        //& is and takes in two symbols
        // 2/ any int is the number of 9's in the discount factor
        // F is eventually or finally
        // G is always
        // prefix notation
        // ! is not

//		formula = "U6!BR"; // avoid blue en route to red
        grid = 'R'; // R N
//		 formula = "U6!BB"; // avoid blue en route to blue
//        formula = "&G4!BF4R"; // eventually red and always not blue (waits)
//		formula = "F3B"; // go to blue
//		formula = "F2R"; // go to red
//		formula = "G3!B"; // avoid blue

//		grid = 'H';
//		formula = "&F3RG1F1B"; // hallway!

//		grid = 'R';
		formula = "U4!BR"; // avoid blue en route to red
//		formula = "G2!B";
//		formula = "G4F4&BF4R"; // slides
//		formula = "&F2RG2!B";  // slides
//		formula = "F5R";
//		formula = "F3G3B";
//		formula = "G4&!R!B";

//		formula = "!F3!F3B"; // always eventually blue
//		formula = "F3&RF3B"; // go to red then blue
//		formula = "F3&BF3R"; // go to blue then red
//		formula = "F3&RF3&BF3R"; // red, blue, red
//		formula = "G3&F3RF3B";    // has to touch both
//		formula = "G1F1R";		// always eventually redformula
//		formula = "F1G1R";		// eventually always red (ever so slightly different)
//		formula = "G4F4B"; // stay in blue (always eventually) (also slow)
//		formula = "G3F3&RF3B"; // red, blue, repeat (planning takes awhile)
//		formula = "F1F1R";	// eventually-eventually...
//		formula = "F2R";	// eventually... same as above!
//		formula = "G2&F2RF2B"; // always go to red and always go to blue (does red-red-blue-blue-red-red-blue-blue...)
//		formula = "&G1F2RG1F2B"; // does a bunch of blue then a bunch of red. not sure why it doesn't intermingle
//		formula = "|G1F1RG4F4B"; // Formula from Charles' talk
//		formula = "&G1F1RG4F4B"; // try "and"
//		formula = "&F1RG1F1B"; // hallway with short time to graduate.
//		formula = "&F1RG1!B"; // avoid blue
//		formula = "U3!BR"; // not blue until red
        // formula = "F3|BR"; // get to blue or red
//		formula = "F3R"; // get to red (ignoring blue)
//		formula = "&F3RG3!B"; // get to red (avoiding blue)
        //formula = "G3!|BR"; // avoid blue and red


        // formula = eventually(make(P))
        // GLTLCompiler.TransitionQuery formulaobj = new GLTLCompiler.TransitionQuery()

        if(args.length > 0){
            formula = args[0];
        }

        System.out.println("Running for formula " + formula);

        GridWorldDomain gwd;
        final OOSADomain envDomain;
        int numLocations;
        GridWorldState s;
        List<GridLocation> locations;
        GridAgent agent;


        switch (grid) {
            case 'R': // Rescue.
                //define our environment MDP
                // GridWorldDomain gwd = new GridWorldDomain(11, 11);
                gwd = new GridWorldDomain(10, 10);
                //gwd.setMapToFourRooms();
                gwd.setNumberOfLocationTypes(2); //two kinds of locations to specify
                gwd.setProbSucceedTransitionDynamics(0.80);

                envDomain = gwd.generateDomain();

                //construct our state
                numLocations = 24; // allocate locations

                agent = new GridAgent(1,7);
                locations = new ArrayList<>();

//                s = GridWorldDomain.getOneAgentNLocationState(envDomain, numLocations);
//                GridWorldDomain.setAgent(s, 1, 7); //agent starting place
                // GridWorldDomain.setAgent(s, 1, 1); //agent starting place initial location is blue
                locations.add(new GridLocation(7,8,1,LOCATIONPREFIX+0));

                int i;
                for (i = 0; i < 3; i++) {
                    locations.add(new GridLocation(i+1,1,0,LOCATIONPREFIX+(i+1)));
//                    GridWorldDomain.setLocation(s, i + 1, i + 1, 1, 0); //second location (1) in 1,1 with type 0
                    // System.out.println("Count is: " + i);
                }
                for (i = 0; i < 4; i++) {
                    locations.add(new GridLocation(i+4,2,0,LOCATIONPREFIX+(i+4)));
//                    GridWorldDomain.setLocation(s, i + 4, i + 4, 2, 0);
                }
                for (i = 0; i < 7; i++) {
                    locations.add(new GridLocation(3,i+3,0,LOCATIONPREFIX+(i+8)));
//                    GridWorldDomain.setLocation(s, i + 8, 3, i + 3, 0);
                }
                for (i = 0; i < 4; i++) {
                    locations.add(new GridLocation(5,i+5,0,LOCATIONPREFIX+(i+15)));
//                    GridWorldDomain.setLocation(s, i + 15, 5, i + 5, 0);
                }
                for (i = 0; i < 3; i++) {
                    locations.add(new GridLocation(7 + i, 5 + i,0,LOCATIONPREFIX+(i+19)));
//                    GridWorldDomain.setLocation(s, i + 19, 7 + i, 5 + i, 0);
                }
                locations.add(new GridLocation(6,5,0,LOCATIONPREFIX+22));
                locations.add(new GridLocation(9,8,0,LOCATIONPREFIX+23));
//                GridWorldDomain.setLocation(s, 22, 6, 5, 0);
//                GridWorldDomain.setLocation(s, 23, 9, 8, 0);
                s = new GridWorldState(agent,locations);

                // State initialEnvState = GridWorldDomain.getOneAgentNoLocationState(envDomain, 0, 0);
                break;
            case 'H': // hallway
                //define our environment MDP
                // GridWorldDomain gwd = new GridWorldDomain(11, 11);
                gwd = new GridWorldDomain(5, 5);
                //gwd.setMapToFourRooms();
                gwd.setNumberOfLocationTypes(2); //two kinds of locations to specify
                //gwd.setProbSucceedTransitionDynamics(0.80);
                gwd.horizontalWall(0, 4, 2); // upper wall
                gwd.horizontalWall(1,1,0); // internal wall
                gwd.horizontalWall(3,3,0); // internal wall

                envDomain = gwd.generateDomain();

                //construct our state
                numLocations = 3; // allocate locations
//                s = GridWorldDomain.getOneAgentNLocationState(envDomain, numLocations);
                agent = new GridAgent(0,1);
                locations = new ArrayList<>();
//                GridWorldDomain.setAgent(s, 0, 1); //agent starting place
                locations.add(new GridLocation(0,0,0,LOCATIONPREFIX+0));// type 1 blue!
                locations.add(new GridLocation(2,0,0,LOCATIONPREFIX+1));
                locations.add(new GridLocation(4,1,1,LOCATIONPREFIX+2));// goal of type 2
//              GridWorldDomain.setLocation(s, 0, 0, 0, 0); //first location (0) with type 1 = blue
//                GridWorldDomain.setLocation(s, 1, 2, 0, 0); //second location with type 1 = blue
//
//                GridWorldDomain.setLocation(s, 2, 4, 1, 1); // goal
                s = new GridWorldState(agent,locations);

                break;

            case 'N': // Russell Norvig grid 3x4
            default:
                //define our environment MDP
                // GridWorldDomain gwd = new GridWorldDomain(11, 11);
                gwd = new GridWorldDomain(4, 4);

                gwd.setNumberOfLocationTypes(2); //two kinds of locations to specify
                gwd.horizontalWall(0,3,3); // upper wall
                gwd.horizontalWall(1,1,1); // internal wall


                double slipProb = 0.2;
                double stayProb = 1.0 - slipProb;
                double thisSlip = slipProb * .5;
                double[][] transitionDynamics = new double[][]
                        {
                                // North   South     East      West
                                {stayProb, 0., thisSlip, thisSlip}, //selecting north
                                {0., stayProb, thisSlip, thisSlip}, //selecting south
                                {thisSlip, thisSlip, stayProb, 0.},       //selecting east
                                {thisSlip, thisSlip, 0., stayProb}  // selecting west
                        };
                gwd.setTransitionDynamics(transitionDynamics);
                envDomain = gwd.generateDomain();

                //construct our state
                numLocations = 2; // allocate locations
                locations = new ArrayList<>();
                agent = new GridAgent(0,0);
                locations.add(new GridLocation(3,1,0,LOCATIONPREFIX+0));
                locations.add(new GridLocation(3,2,1,LOCATIONPREFIX+1));
                s = new GridWorldState(agent,locations);
//                s = GridWorldDomain.getOneAgentNLocationState(envDomain, numLocations);
//                GridWorldDomain.setAgent(s, 2, 1); //agent starting place
//                GridWorldDomain.setLocation(s, 0, 3, 1, 0); //first location (0) with type 1 = goal
//                GridWorldDomain.setLocation(s, 1, 3, 2, 1); //second location (1) with type 1 = blue
                break;
        }

        //add propositional function for checking if the agent is in an orange or blue cell
        PropositionalFunction inOrange = new InOrange(PFINORANGE, new String [] {});
        PropositionalFunction inBlue = new InBlue(PFINBLUE, new String [] {});
        PropositionalFunction notInBlue = new NotInBlue(PFNOTINBLUE, new String [] {});

//        new InBlue(envDomain);
//        new NotInBlue(envDomain);

        //let our GLTL symbol "o" correspond to evaluating whether the agent is in an orange location (a parameterless propositional function)
        // and "b" be whether the agent is in a blue location
        Map<String, GroundedProp> symbolMap = new HashMap<String, GroundedProp>(1);
        //System.out.println(envDomain.getPropFunctions());
        symbolMap.put("R", new GroundedProp(inOrange, new String[]{}));
        symbolMap.put("B", new GroundedProp(inBlue, new String[]{}));
        // symbolMap.put("R", envDomain.getPropFunction(PFNOTINBLUE));
        //System.out.println(symbolMap.toString());


        //construct our GLTL compiler for our grid world environment and the given formula (currently hardcoded by numbers)
        GLTLCompiler compiler = new GLTLCompiler(formula, symbolMap, envDomain);
        OOSADomain compiledDomain = compiler.generateDomain();
//        RewardFunction rf = compiler.generateRewardFunction();
//        TerminalFunction tf = compiler.generateTerminalFunction();

        State initialCompiledState = compiler.addInitialTaskStateToEnvironmentState(s);
        List<ActionType> atTypes = compiledDomain.getActionTypes();

        // System.out.println(initialCompiledState.getCompleteStateDescription());

        //begin planning in our compiled domain
        ValueIteration vi = new ValueIteration(compiledDomain, 1.0, new SimpleHashableStateFactory(), 0.0000001, 20000);
        vi.toggleReachabiltiyTerminalStatePruning(true);
//		System.out.println(initialCompiledState.toString());
//		System.out.println(vi.getQ(initialCompiledState, compiledDomain.getSingleAction("N")));
        vi.planFromState(initialCompiledState);
        Environment env = new SimulatedEnvironment(compiledDomain, initialCompiledState);
        Policy p = new GreedyQPolicy(vi);
        Episode ea = PolicyUtils.rollout(p,env);

        StateEnumerator senum = new StateEnumerator(compiledDomain,new SimpleHashableStateFactory());

        senum.findReachableStatesAndEnumerate(initialCompiledState);


        System.out.println("print states start");
//        List<State> states = vi.getAllStates();
        for(int i=0;i< senum.numStatesEnumerated();i++){
            State state = senum.getStateForEnumerationId(i);
            System.out.println("|||||||||||||");
            System.out.println(stateToStringCompact(state));
            for(ActionType at:atTypes){
                List<Action> actions = at.allApplicableActions(state);
                String str = "";
                for(Action a:actions){
                    str +=a.actionName() + " ";

                }
                System.out.println(str);
            }
        }
        System.out.println("print states end");
        for(ActionType at:atTypes){
            List<Action> actions = at.allApplicableActions(initialCompiledState);
            for(Action a:actions){
                System.out.println(a.actionName());
            }
        }

        for(ActionType at:atTypes){
            List<Action> actions = at.allApplicableActions(initialCompiledState);
            for(Action a:actions){
                EnvironmentOutcome eo = compiledDomain.getModel().sample(initialCompiledState,a);
                System.out.println(eo.op.toString());
                System.out.println("---------------++++++++++++++++-------------------------+++++++");
            }
        }



//        for(int count = 0;count<ea.numActions();count++){
//            GridWorldState gs = (GridWorldState)((GLTLState)ea.stateSequence.get(count)).envState;
//
//            System.out.println(gs.toString() + "////////////////" + ea.actionSequence.get(count));
//            System.out.println("_____________________");
//        }

        Episode gw_episode= new Episode((GridWorldState)((GLTLState)ea.stateSequence.get(0)).envState);

        for(int count = 0;count<ea.numActions();count++){
//            GridWorldState gs = (GridWorldState)((GLTLState)ea.stateSequence.get(count)).envState;
            GridWorldState gsNew = (GridWorldState)((GLTLState)ea.stateSequence.get(count+1)).envState;


//            EnvironmentOutcome eo = new EnvironmentOutcome(gs,ea.actionSequence.get(count),gsNew,0,false);
            gw_episode.transition(ea.actionSequence.get(count),gsNew,0);
//            System.out.println("_____________________");
        }



        System.out.println("num actions: " + ea.numActions());

            Visualizer v = GridWorldVisualizer.getVisualizer(gwd.getMap());
//        StaticPainter sp = new StaticPainter() {
//            @Override
//            public void paint(Graphics2D g2, State s, float cWidth, float cHeight) {
//                ObjectInstance taskObject = s.getFirstObjectOfClass(GLTLCompiler.CLASSSPEC);
//                if(taskObject != null) {
//                    System.out.println(taskObject.getObjectDescription());
//                }
//            }
//        };
//        v.addStaticPainter(sp);
        List<Episode> episodes = new ArrayList<>();
        episodes.add(gw_episode);
        new EpisodeSequenceVisualizer(v,envDomain,episodes);
//		new EpisodeSequenceVisualizer(v, gwd, episodes);
//
//		//create episode sequence visualizer
//		Visualizer v = GridWorldVisualizer.getVisualizer(gwd.getMap());
//        EpisodeSequenceVisualizer evis = new EpisodeSequenceVisualizer(v, compiledDomain, Arrays.asList(ea));
////		EpisodeRenderer evis = new EpisodeRenderer(v, compiledDomain, Arrays.asList(ea));

    }



    public static class InOrange extends PropositionalFunction {

        //        public InOrange(Domain domain) {
//            super(PFINORANGE, domain, "");
//        }
        public InOrange(String name, String [] params) {
            super(name, params);
        }


        @Override
        public boolean isTrue(OOState ooState, String... strings) {
            GridAgent agent = ((GridWorldState)ooState).agent;
            int x = agent.x;
            int y = agent.y;
            List<GridLocation> locations = ((GridWorldState)ooState).locations;
            for(GridLocation location:locations){
                int lx = location.x;
                int ly = location.y;

                if ((lx == x) && (ly == y) && (location.type == 1)) {
                    return true;
                }
            }
            return false;
        }
    }

    public static class InBlue extends PropositionalFunction {

        public InBlue(String name, String [] params) {
            super(name, params);
        }

//        public InBlue(Domain domain) {
//            super(PFINBLUE, domain, "");
//        }

        //        @Override
//        public boolean isTrue(State s, String[] params) {
//            ObjectInstance agent = s.getFirstObjectOfClass(GridWorldDomain.CLASSAGENT);
//            int x = agent.getIntValForAttribute(GridWorldDomain.ATTX);
//            int y = agent.getIntValForAttribute(GridWorldDomain.ATTY);
//
//            List<ObjectInstance> locations = s.getObjectsOfClass(GridWorldDomain.CLASSLOCATION);
//            for (ObjectInstance location : locations) {
//                int lx = location.getIntValForAttribute(GridWorldDomain.ATTX);
//                int ly = location.getIntValForAttribute(GridWorldDomain.ATTY);
//
//                if ((lx == x) && (ly == y) && (location.getIntValForAttribute(GridWorldDomain.ATTLOCTYPE) == 0)) {
//                    return true;
//                }
//            }
//            return false;
//        }
        @Override
        public boolean isTrue(OOState ooState, String... strings) {
            GridAgent agent = ((GridWorldState)ooState).agent;
            int x = agent.x;
            int y = agent.y;
            List<GridLocation> locations = ((GridWorldState)ooState).locations;
            for(GridLocation location:locations){
                int lx = location.x;
                int ly = location.y;

                if ((lx == x) && (ly == y) && (location.type == 0)) {
                    return true;
                }
            }
            return false;
        }

    }

    public static class NotInBlue extends PropositionalFunction {

        public NotInBlue(String name, String [] params) {
            super(name, params);
        }

//        public NotInBlue(Domain domain) {
//            super(PFNOTINBLUE, domain, "")
//        }

//        @Override
//        public boolean isTrue(State s, String[] params) {
//            ObjectInstance agent = s.getFirstObjectOfClass(GridWorldDomain.CLASSAGENT);
//            int x = agent.getIntValForAttribute(GridWorldDomain.ATTX);
//            int y = agent.getIntValForAttribute(GridWorldDomain.ATTY);
//
//            List<ObjectInstance> locations = s.getObjectsOfClass(GridWorldDomain.CLASSLOCATION);
//            for (ObjectInstance location : locations) {
//                int lx = location.getIntValForAttribute(GridWorldDomain.ATTX);
//                int ly = location.getIntValForAttribute(GridWorldDomain.ATTY);
//
//                if ((lx == x) && (ly == y) && (location.getIntValForAttribute(GridWorldDomain.ATTLOCTYPE) == 0)) {
//                    return false;
//                }
//            }
//            return true;
//        }

        @Override
        public boolean isTrue(OOState ooState, String... strings) {
            GridAgent agent = ((GridWorldState)ooState).agent;
            int x = agent.x;
            int y = agent.y;
            List<GridLocation> locations = ((GridWorldState)ooState).locations;
            for(GridLocation location:locations){
                int lx = location.x;
                int ly = location.y;

                if ((lx == x) && (ly == y) && (location.type == 0)) {
                    return false;
                }
            }
            return true;
        }

    }

    /**
     * Modified version of burlap.mdp.core.state.StateUtilities.
     * Returns single-line representation of state, easier to parse.
     * @param sIn
     * @return
     */
    public static String stateToStringCompact(State sIn){
        GLTLState s = (GLTLState) sIn;
        StringBuilder buf = new StringBuilder();
        //buf.append("{");
        List<Object> keys = s.envState.variableKeys();
        for(Object key : keys){
            buf.append(key.toString()).append(": {").append(s.envState.get(key).toString()).append("} ");
        }
        buf.append("spec: "+ s.spec);
        return buf.toString();
    }
}