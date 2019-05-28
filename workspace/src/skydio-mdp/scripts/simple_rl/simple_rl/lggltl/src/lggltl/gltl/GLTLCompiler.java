package lggltl.gltl;

import burlap.mdp.auxiliary.DomainGenerator;
import burlap.mdp.core.Domain;
import burlap.mdp.core.TerminalFunction;
import burlap.mdp.core.action.Action;
import burlap.mdp.core.action.ActionType;
import burlap.mdp.core.oo.propositional.GroundedProp;
import burlap.mdp.core.oo.state.OOState;
import burlap.mdp.core.state.State;
import burlap.mdp.singleagent.model.FactoredModel;
import burlap.mdp.singleagent.model.RewardFunction;
import burlap.mdp.singleagent.model.statemodel.FullStateModel;
import burlap.mdp.singleagent.model.statemodel.SampleStateModel;
import burlap.mdp.singleagent.oo.OOSADomain;
import lggltl.gltl.state.GLTLState;

import java.util.*;

/**
 * @author James MacGlashan, reworked by Michael Littman, rereworked by Dilip Arumugam, rerereworked by nakul gopalan
 */

//TODO: the env. state needs to be an OOState for this domain because Prop functions are OO that seems stupid!!
//TODO: the env. needs to have a full model and not just a sample model in this code!
public class GLTLCompiler implements DomainGenerator {

    /** Name for OO-MDP class denoting GLTL spec MDP state **/
    public static final String CLASSSPEC = "##spec";

    /** Integer attribute representing state of GLTL spec MDP (init, acc, rej) **/
    public static final String ATTSPEC = "##spec";

    /** Compiled action type name **/
    public static  final String ACTION_COMPILED = "compiledActionType";


    /** Environment MDP that we would like to act in **/
    protected OOSADomain environmentDomain;

    /** GLTL formula representing the task to complete within the environment **/
    protected String formula;

    /** Object for managing GLTL transitions **/
    protected TransitionQuery transitionQuery;

    /** Object for holding propositional functions that underlie GLTL formula atoms **/
    protected SymbolEvaluator symbolEvaluator;


    public GLTLCompiler(String formula, Map<String, GroundedProp> symbolMap, OOSADomain environmentDomain) {
        this.formula = formula;
        this.symbolEvaluator = new SymbolEvaluator(symbolMap);
        this.environmentDomain = environmentDomain;
    }

    public String getFormula() {
        return this.formula;
    }

    public void setFormula(String formula, Map<String, GroundedProp> symbolMap) {
        this.formula = formula;
        this.symbolEvaluator = new SymbolEvaluator(symbolMap);
    }

    public Domain getEnvironmentDomain() {
        return environmentDomain;
    }

    public void setEnvironmentDomain(OOSADomain environmentDomain) {
        this.environmentDomain = environmentDomain;
    }

    public OOState addInitialTaskStateToEnvironmentState(OOState environmentState) {

//        MutableOOState newState = (MutableOOState) environmentState.copy();
//
//        //first remove any task spec objects if they are there
//        for (ObjectInstance ob : newState.objectsOfClass(CLASSSPEC)) {
//            newState.removeObject(ob.name());
//        }
//
//
//        ObjectInstance taskOb = new ObjectInstance(compiledDomain.stateClass(CLASSSPEC), CLASSSPEC);
//
//        taskOb.setValue(ATTSPEC, 2);
//
//        newState.addObject(taskOb);

        return new GLTLState(environmentState);

    }

    @Override
    public OOSADomain generateDomain() {


        OOSADomain domain = new OOSADomain();

        domain.addStateClass(CLASSSPEC, GLTLState.class);

        FactoredModel model = new FactoredModel(new GLTLModel((FullStateModel)((FactoredModel)this.environmentDomain.getModel()).getStateModel()), this.generateRewardFunction(), this.generateTerminalFunction());


        domain.setModel(model);

        TransitionQuery transitionQuery = TransitionQuery.compileFormula(domain, this.formula, 0);


//      DUMP GRAPH for debugging
        //TODO: graph does not look like this!
        System.out.println(transitionQuery.symbolMap);
        for (Map.Entry<IntegerPair, List<TaskMDPTransition>> e : transitionQuery.transitions.entrySet()) {
            System.out.println(e.getKey().a + ", " + e.getKey().b);
            for (TaskMDPTransition trans : e.getValue()) {
                System.out.println(trans.p + "->" + trans.task);
            }
        }
        System.out.println("=========");

        List<ActionType> actionTypes = this.environmentDomain.getActionTypes();

        for (ActionType a : actionTypes) {
            domain.addActionType(new CompiledActionType(a, domain,
                    this.formula, this.symbolEvaluator, transitionQuery));
        }

        return domain;

    }


    public RewardFunction generateRewardFunction() {

        return new RewardFunction() {
            @Override
            public double reward(State s, Action action, State sprime) {
                int specOld = ((GLTLState)s).spec;

                int specNew = ((GLTLState)sprime).spec;

                if (specNew == 1 && !(specOld == 1)) {
                    return 1.;
                } else if (specNew == 0) {
                    return 0;
                } else {
                    return 0;
                }
            }
        };

    }

    public TerminalFunction generateTerminalFunction() {
        return new TerminalFunction() {
            @Override
            public boolean isTerminal(State s) {
                int as = ((GLTLState)s).spec;

                return ((as == 1) || (as == 0));
            }
        };
    }


    public static class CompiledActionType implements ActionType {

        protected ActionType srcActionType;
        protected String formula;
        protected SymbolEvaluator symbolEvaluator;
        protected TransitionQuery transitions;
        protected FullStateModel model;


        public CompiledActionType(ActionType srcActionType, OOSADomain domain, String formula, SymbolEvaluator symbolEvaluator, TransitionQuery transitions) {
//            super(srcType.typeName(), domain, srcAction.getParameterClasses(), srcAction.getParameterOrderGroups());
            this.srcActionType = srcActionType;
            this.formula = formula;
            this.symbolEvaluator = symbolEvaluator;
            this.transitions = transitions;
            this.model = (FullStateModel)((FactoredModel)domain.getModel()).getStateModel();
        }

        public SymbolEvaluator getSymbolEvaluator() {
            return symbolEvaluator;
        }

        public String getFormula() {
            return formula;
        }



//        public List<StateTransitionProb> getTransitions(State s, String[] params) {
//
//            //get the environment mdp transition dynamics
//            List<StateTransitionProb> environmentTPs = this.model.stateTransitions(s, this.srcActionType);
//
//            //reserve space for the joint task-environment mdp transitions
//            List<StateTransitionProb> jointTPs = new ArrayList<StateTransitionProb>(environmentTPs.size() * 2);
//
//            //perform outer loop of transitions cross product over environment transitions
//            for (StateTransitionProb etp : environmentTPs) {
//
//                //get the task transitions and expand them with the environment transition
//                List<TaskMDPTransition> taskTPs = this.getTaskTransitions(s, etp.s);
////				System.out.println("===>" + taskTPs.size());
//                double taskSum = 0.;
//                for (TaskMDPTransition ttp : taskTPs) {
//                    State ns = etp.s.copy();
//                    //remove the old task spec
//                    ns.removeObject(ns.getFirstObjectOfClass(CLASSSPEC).getName());
//                    //set the new task spec
//                    ns.addObject(ttp.taskObject);
//                    double p = etp.p * ttp.p;
//                    StateTransitionProb jtp = new StateTransitionProb(ns, p);
//                    jointTPs.add(jtp);
//
//                    taskSum += ttp.p;
//                }
//
//                if (Math.abs(taskSum - 1.) > 1e-5) {
//                    throw new RuntimeException("Error, could not return transition probabilities because task MDP transition probabilities summed to " + taskSum + " instead of 1.");
//                }
//
//            }
//
//
//            return jointTPs;
//        }
// We want to know, given a pair of s and s', what's the probability of making a transition
// from s to s'.

        protected List<TaskMDPTransition> getTaskTransitions(State s, State nextEnvState) {

            GLTLState gs = (GLTLState)s;

            GLTLState ss = gs.copy();

//            ObjectInstance agentSpec = s.getFirstObjectOfClass(CLASSSPEC);

            int curStateSpec = gs.spec; // task state (for s)

            List<String> dependencies = this.transitions.symbolDependencies(curStateSpec);

            //TODO: can we compare this to a point in the paper. IDK what is going on here!!!!
            int actionlabel = 0;
//            System.out.println(curStateSpec + "\t" + dependencies);
            for (String symbolName : dependencies) {
                actionlabel = 2 * actionlabel + ((this.symbolEvaluator.eval(symbolName, (OOState) nextEnvState)) ? 1 : 0);
            }

//			System.out.println(curStateSpec + "/" + actionlabel + ">" + dependencies);
//			System.out.println("::::" + this.transitions.nextTaskStateTransitions(curStateSpec, actionlabel).size());
            return this.transitions.nextTaskStateTransitions(curStateSpec, actionlabel);
        }

//        @Override
//        protected State performActionHelper(State s, String[] params) {
//
//            //sample an environment state
//            State environmentNextState = this.srcAction.performAction(s, params);
//
//            //determine the next task state distribution and sample from it
//            List<TaskMDPTransition> taskTPs = this.getTaskTransitions(s, environmentNextState);
//            double r = RandomFactory.getMapped(0).nextDouble();
//            double sumP = 0.;
//            for (TaskMDPTransition ttp : taskTPs) {
//                sumP += ttp.p;
//                if (r < sumP) {
//                    //remove the old task spec
//                    environmentNextState.removeObject(environmentNextState.getFirstObjectOfClass(CLASSSPEC).getName());
//                    //set the new task spec
//                    environmentNextState.addObject(ttp.taskObject);
//                    return environmentNextState;
//                }
//            }
//
//            throw new RuntimeException("Could not sample action from " + this.getName() + " because the task transition dynamics did not sum to 1.)");
//        }

//        @Override
//        public boolean parametersAreObjects() {
//            return this.srcAction.parametersAreObjects();
//        }

//        @Override
//        public boolean applicableInState(State s, String[] params) {
//            return this.srcAction.applicableInState(s, params);
//        }


        @Override
        public List<Action> allApplicableActions(State s) {
            List<Action> srcGas = this.srcActionType.allApplicableActions(((GLTLState) s).getBaseState());
            List<Action> targetGas = new ArrayList<Action>(srcGas.size());
            for (Action ga : srcGas) {
                targetGas.add(new CompiledAction(ga, this));
            }
            return targetGas;
        }

//        @Override
//        public List<GroundedAction> allApplicableActions(State s) {
//            List<GroundedAction> srcGas = this.srcAction.getAllApplicableGroundedActions(s);
//            List<GroundedAction> targetGas = new ArrayList<GroundedAction>(srcGas.size());
//            for (GroundedAction ga : srcGas) {
//                targetGas.add(new GroundedAction(this, ga.params));
//            }
//
//            return targetGas;
//        }

        @Override
        public String typeName() {
            return ACTION_COMPILED +  srcActionType.toString();
        }

        @Override
        public Action associatedAction(String s) {
            // this is missing as this method is being used on shell, I do not think we need it
            // and we do not have a way to create a wrapped action when do not know which source action.
            throw new RuntimeException("There are no associated parameterized actions.");
        }

    }


    //TODO: the equality and hashcode should check for the environment action too!!


    public static class SymbolEvaluator {

        protected Map<String, GroundedProp> symbolMapping;

        public SymbolEvaluator(Map<String, GroundedProp> symbolMapping) {
            this.symbolMapping = symbolMapping;
        }

        public Map<String, GroundedProp> getSymbolMapping() {
            return symbolMapping;
        }

        public void setSymbolMapping(Map<String, GroundedProp> symbolMapping) {
            this.symbolMapping = symbolMapping;
        }

        public boolean eval(String symbol, OOState s) {
            GroundedProp gp = this.symbolMapping.get(symbol);
            if (gp == null) {
                throw new RuntimeException("Symbol " + symbol + " cannot be evaluated because it is undefined");
            }
            return gp.isTrue(s);
        }
    }

    /** This data structure stores the transitions from one task type to another **/
    protected static class TaskMDPTransition {

        public int task;
        public double p;

        public TaskMDPTransition(int task, double p) {
            this.task = task;
            this.p = p;
        }
    }


    /** This data structure stores a pair of integers for task switching probabilities **/
    protected static class IntegerPair {
        public int a;
        public int b;

        public IntegerPair(int a, int b) {
            this.a = a;
            this.b = b;
        }

        @Override
        public int hashCode() {
            return a + 31 * b;
        }

        @Override
        public boolean equals(Object obj) {
            IntegerPair p = (IntegerPair) obj;
            return (a == p.a) && (b == p.b);
        }
    }

    // Data structure for holding task transitions.
    public static class TransitionQuery {

        Map<Integer, List<String>> symbolMap = new HashMap<>();
        Map<IntegerPair, List<TaskMDPTransition>> transitions = new HashMap<>();

        public static TransitionQuery compileFormula(Domain domain, String formula, int pos) {
            TransitionQuery subexp;
            TransitionQuery subexp2;
            int k;
            double discount;

            switch (formula.charAt(pos)) {
                case '!':
                    subexp= compileFormula(domain, formula, pos + 1);
                    return subexp.not();
                case 'F':
                    subexp = compileFormula(domain, formula, pos + 2);
                    k = (int) (formula.charAt(pos+1)-'0');
                    discount = 1.0-Math.pow(.1, (double) k);
                    return subexp.eventually(discount);
                case 'G':
                    subexp = compileFormula(domain, formula, pos + 2);
                    subexp = subexp.not();
                    k = (int) (formula.charAt(pos+1)-'0');
                    discount = 1.0-Math.pow(.1, (double) k);
                    subexp = subexp.eventually(discount);
                    return subexp.not();
                case 'U':
                    subexp = compileFormula(domain, formula, pos + 2);
                    k = (int) (formula.charAt(pos+1)-'0');
                    discount = 1.0-Math.pow(.1, (double) k);
                    subexp2 = compileFormula(domain, formula, skipFormula(formula, pos + 2));
                    return subexp.until(discount, subexp2);
                case '&':
                    subexp = compileFormula(domain, formula, pos + 1);
                    subexp2 = compileFormula(domain, formula, skipFormula(formula, pos + 1));
                    return subexp.and(subexp2);
                case '|':
                    subexp = compileFormula(domain, formula, pos + 1);
                    subexp2 = compileFormula(domain, formula, skipFormula(formula, pos + 1));
                    subexp = subexp.not();
                    subexp2 = subexp2.not();
                    subexp = subexp.and(subexp2);
                    return subexp.not();
                default:
                    List<String> dependencies = Arrays.asList("" + formula.charAt(pos));
                    TransitionQuery var = new TransitionQuery();
                    var.symbolMap.put(2, dependencies);
                    List<TaskMDPTransition> succeed = new ArrayList<>();
                    int o =  1;
                    succeed.add(new TaskMDPTransition(o, 1.0));
                    var.transitions.put(new IntegerPair(2, 1), succeed);

                    List<TaskMDPTransition> fail = new ArrayList<>();
                    int fo = 0;
                    fail.add(new TaskMDPTransition(fo, 1.0));
                    var.transitions.put(new IntegerPair(2, 0), fail);
                    return var;
            }
        }

        // Where is the next place to translate?
        public static int skipFormula(String formula, int pos) {
            switch (formula.charAt(pos)) {
                case 'F':
                case 'G':
                    return skipFormula(formula, pos + 2);
                case '!':
                    return skipFormula(formula, pos + 1);
                case '&':
                case '|':
                    return skipFormula(formula, skipFormula(formula, pos + 1));
                case 'U':
                    return skipFormula(formula, skipFormula(formula, pos + 2));
                default:
                    return pos + 1;
            }
        }

        public void setTransitions(int taskstate, int actionlabel, List<TaskMDPTransition> transitionList) {
            transitions.put(new IntegerPair(taskstate, actionlabel), transitionList);
        }


        public void setDependency(int taskstate, List<String> dependencies) {
            symbolMap.put(taskstate, dependencies);
        }

        public List<String> symbolDependencies(int taskstate) {
            //TODO: Marking the change here
//            if(symbolMap.containsKey(taskstate)){
//                return symbolMap.get(taskstate);
//            }
//            else{
//                return new ArrayList<>();
//            }
            return symbolMap.get(taskstate);
        }

        public List<TaskMDPTransition> nextTaskStateTransitions(int taskstate, int actionlabel) {
            //TODO: Marking the change here
//            IntegerPair ip = new IntegerPair(taskstate, actionlabel);
//            if(transitions.containsKey(ip)){
//                return transitions.get(ip);
//            }
//            else{
//                return new ArrayList<>();
//            }
            return transitions.get(new IntegerPair(taskstate, actionlabel));
        }

        public TransitionQuery not() {
            TransitionQuery negated = new TransitionQuery();
            negated.symbolMap = new HashMap<>(this.symbolMap);
            negated.transitions = new HashMap<>(this.transitions.size());

            for (Map.Entry<IntegerPair, List<TaskMDPTransition>> e : this.transitions.entrySet()) {
                List<TaskMDPTransition> negatedTransitions = new ArrayList<TaskMDPTransition>(e.getValue().size());
                for (TaskMDPTransition trans : e.getValue()) {
                    if (trans.task == 0) {
                        int newtrans = 1;
                        negatedTransitions.add(new TaskMDPTransition(newtrans, trans.p));
                    } else if (trans.task == 1) {
                        int newtrans = 0;
                        negatedTransitions.add(new TaskMDPTransition(newtrans, trans.p));
                    } else {
                        negatedTransitions.add(new TaskMDPTransition(trans.task, trans.p));
                    }
                }
                negated.transitions.put(e.getKey(), negatedTransitions);
            }
            return negated;
        }

        public int getMaximumState() {
            int max = -1;

            for (Map.Entry<Integer, List<String>> e : this.symbolMap.entrySet()) {
                int id = e.getKey();

                max = Math.max(id, max);
            }
            return max;
        }

        // Let subexpression finish if it runs over time.
        public TransitionQuery eventuallyOLD(double discount) {
            TransitionQuery constructed = new TransitionQuery();
            constructed.symbolMap = new HashMap<>(this.symbolMap);
            constructed.transitions = new HashMap<>(this.transitions.size());

            int baseline = getMaximumState() + 1;

            // set symbol map for new states
            for (Map.Entry<Integer, List<String>> e : this.symbolMap.entrySet()) {
                int state = e.getKey();
                List<String> dependencies = e.getValue();
//				System.out.println(state+baseline + "+" + dependencies);
                if (state > 1) {
                    constructed.symbolMap.put(state + baseline, dependencies);
                }
            }

            for (Map.Entry<IntegerPair, List<TaskMDPTransition>> e : this.transitions.entrySet()) {
                List<TaskMDPTransition> constructedTransitions1 = new ArrayList<TaskMDPTransition>(e.getValue().size());
                List<TaskMDPTransition> constructedTransitions2 = new ArrayList<TaskMDPTransition>(e.getValue().size());

                // construct new destinations
                for (TaskMDPTransition trans : e.getValue()) {
                    int olddest = trans.task;
                    if (olddest == 1) {
                        int newtrans = Integer.valueOf(trans.task);
                        constructedTransitions1.add(new TaskMDPTransition(newtrans, trans.p));
                        constructedTransitions2.add(new TaskMDPTransition(newtrans, trans.p));
                    } else if (olddest == 0) {
                        int newtransrestart = 2;
                        int newtransfail = Integer.valueOf(trans.task);
                        int newtransexpire = 2 + baseline;
//                        newtransrestart.setValue(ATTSPEC, 2);
//                        newtransexpire.setValue(ATTSPEC, 2 + baseline);
                        constructedTransitions1.add(new TaskMDPTransition(newtransrestart, trans.p * discount));
                        constructedTransitions1.add(new TaskMDPTransition(newtransexpire, trans.p * (1.0 - discount)));
                        constructedTransitions2.add(new TaskMDPTransition(newtransfail, trans.p));
                    } else {
                        int newtrans1 = Integer.valueOf(trans.task);
                        int newtrans2 = olddest + baseline;
                        constructedTransitions1.add(new TaskMDPTransition(newtrans1, trans.p * discount));
                        constructedTransitions1.add(new TaskMDPTransition(newtrans2, trans.p * (1.0 - discount)));
                        constructedTransitions2.add(new TaskMDPTransition(newtrans2, trans.p));
                    }
                }
                constructed.transitions.put(e.getKey(), constructedTransitions1);
                constructed.transitions.put(new IntegerPair(e.getKey().a + baseline, e.getKey().b), constructedTransitions2);
            }
            return constructed;
        }

        // formula must be true until op2 becomes true.
        public TransitionQuery until(double discount, TransitionQuery op2) {
            TransitionQuery constructed = new TransitionQuery();
            constructed.symbolMap = new HashMap<>(); // this.symbolMap);
            constructed.transitions = new HashMap<>(this.transitions.size());

            int baseline = op2.getMaximumState() + 1;

            // Try all pairs.
            for (Map.Entry<IntegerPair, List<TaskMDPTransition>> e1 : this.transitions.entrySet()) {
                for (Map.Entry<IntegerPair, List<TaskMDPTransition>> e2 : op2.transitions.entrySet()) {
                    for (TaskMDPTransition trans1 : e1.getValue()) {
                        for (TaskMDPTransition trans2 : e2.getValue()) {
                            // get a pair of transitions from the old machines
                            int from1 = e1.getKey().a;
                            int from2 = e2.getKey().a;
                            int actionlabel1 = e1.getKey().b;
                            int actionlabel2 = e2.getKey().b;
                            double prob1 = trans1.p;
                            double prob2 = trans2.p;
                            int to1 = trans1.task;
                            int to2 = trans2.task;

                            // configure the combined transition  for the new machine
                            int from = until_combine(from1, from2, baseline);
                            System.out.println("from ("+from1 + "," + from2+")="+from);
                            int to = until_combine(to1, to2, baseline);
                            System.out.println("to ("+to1 + "," + to2+")="+to);
                            double prob = prob1 * prob2;
                            int actionlabel = actionlabel1 * (1 << op2.symbolMap.get(from2).size()) + actionlabel2;

//							System.out.println(this.symbolMap + "||" + op2.symbolMap);
//							System.out.println(0/0);
                            // Store things in the new machine
                            List<TaskMDPTransition> currentlist = constructed.transitions.get(new IntegerPair(from, actionlabel));
                            if (currentlist == null) currentlist = new ArrayList<>();

//							System.out.println(from + "->" + to + " " + actionlabel + " " + currentlist);
//							System.out.println(from1 + "->" + to1 + ", " + from2 + "->" + to2 + " => " + from + "->" + to);

                            if (to > 1) { // not terminal
                                int trans = Integer.valueOf(to);
                                currentlist.add(new TaskMDPTransition(trans, prob*discount));
                                int transfail = 0;
                                currentlist.add(new TaskMDPTransition(transfail, prob*(1.0-discount)));
                            } else { // terminal
                                int transend = Integer.valueOf(to);
                                currentlist.add(new TaskMDPTransition(transend, prob));
                            }
                            constructed.transitions.put(new IntegerPair(from, actionlabel), currentlist);

                            List<String> dependencies1 = this.symbolDependencies(from1);
                            List<String> dependencies2 = op2.symbolDependencies(from2);
                            List<String> dependencies = new ArrayList<String>(dependencies1);
                            dependencies.addAll(dependencies2);

                            constructed.symbolMap.put(from, dependencies);
                        }
                    }
                }
            }
            return constructed;
        }

        // don't let subexpression finish if it runs over time.
        public TransitionQuery eventually(double discount) {
            TransitionQuery constructed = new TransitionQuery();
            constructed.symbolMap = new HashMap<>(this.symbolMap);
            constructed.transitions = new HashMap<>(this.transitions.size());

            // loop through transitions
            for (Map.Entry<IntegerPair, List<TaskMDPTransition>> e : this.transitions.entrySet()) {
                List<TaskMDPTransition> constructedTransitions = new ArrayList<TaskMDPTransition>(e.getValue().size());

                // construct new destinations
                for (TaskMDPTransition trans : e.getValue()) {
                    int olddest = Integer.valueOf(trans.task);
                    if (olddest == 1) {
                        int newtrans = Integer.valueOf(trans.task);
                        constructedTransitions.add(new TaskMDPTransition(newtrans, trans.p));
                    } else if (olddest == 0) {
                        int newtransrestart = 2;
                        int newtransfail = Integer.valueOf(trans.task);
                        constructedTransitions.add(new TaskMDPTransition(newtransrestart, trans.p * discount));
                        constructedTransitions.add(new TaskMDPTransition(newtransfail, trans.p * (1.0 - discount)));
                    } else {
                        int newtranscont = Integer.valueOf(trans.task);
                        int newtransexpire = 0; // modification
                        constructedTransitions.add(new TaskMDPTransition(newtranscont, trans.p * discount));
                        constructedTransitions.add(new TaskMDPTransition(newtransexpire, trans.p * (1.0 - discount)));
                    }
                }
                constructed.transitions.put(e.getKey(), constructedTransitions);
            }
            return constructed;
        }

        public TransitionQuery and(TransitionQuery op2) {
            TransitionQuery constructed = new TransitionQuery();
            constructed.symbolMap = new HashMap<>(); // this.symbolMap);
            constructed.transitions = new HashMap<>(this.transitions.size());

            int baseline = op2.getMaximumState() + 1;

            // Try all pairs.
            for (Map.Entry<IntegerPair, List<TaskMDPTransition>> e1 : this.transitions.entrySet()) {
                for (Map.Entry<IntegerPair, List<TaskMDPTransition>> e2 : op2.transitions.entrySet()) {
                    for (TaskMDPTransition trans1 : e1.getValue()) {
                        for (TaskMDPTransition trans2 : e2.getValue()) {
                            // get a pair of transitions from the old machines
                            int from1 = e1.getKey().a;
                            int from2 = e2.getKey().a;
                            int actionlabel1 = e1.getKey().b;
                            int actionlabel2 = e2.getKey().b;
                            double prob1 = trans1.p;
                            double prob2 = trans2.p;
                            int to1 = Integer.valueOf(trans1.task);
                            int to2 = Integer.valueOf(trans2.task);

                            // configure the combined transition  for the new machine
                            int from = and_combine(from1, from2, baseline);
                            int to = and_combine(to1, to2, baseline);
                            double prob = prob1 * prob2;
                            int actionlabel = actionlabel1 * (1 << op2.symbolMap.get(from2).size()) + actionlabel2;

//							System.out.println(this.symbolMap + "||" + op2.symbolMap);
//							System.out.println(0/0);
                            // Store things in the new machine
                            List<TaskMDPTransition> currentlist = constructed.transitions.get(new IntegerPair(from, actionlabel));
                            if (currentlist == null) currentlist = new ArrayList<>();

//							System.out.println(from + "->" + to + " " + actionlabel + " " + currentlist);
//							System.out.println(from1 + "->" + to1 + ", " + from2 + "->" + to2 + " => " + from + "->" + to);

                            int trans = Integer.valueOf(to);
                            currentlist.add(new TaskMDPTransition(trans, prob));
                            constructed.transitions.put(new IntegerPair(from, actionlabel), currentlist);

                            List<String> dependencies1 = this.symbolDependencies(from1);
                            List<String> dependencies2 = op2.symbolDependencies(from2);
                            List<String> dependencies = new ArrayList<String>(dependencies1);
                            dependencies.addAll(dependencies2);

                            constructed.symbolMap.put(from, dependencies);
                        }
                    }
                }
            }

            // from1 = 1, to1 = 1
            for (Map.Entry<IntegerPair, List<TaskMDPTransition>> e2 : op2.transitions.entrySet()) {
                for (TaskMDPTransition trans2 : e2.getValue()) {
                    // get a pair of transitions from the old machines
                    int from1 = 1;
                    int from2 = e2.getKey().a;
                    int actionlabel = e2.getKey().b;
                    double prob = trans2.p;
                    int to1 = 1;
                    int to2 = Integer.valueOf(trans2.task);

                    // configure the combined transition  for the new machine
                    int from = and_combine(from1, from2, baseline);
                    int to = and_combine(to1, to2, baseline);

                    // Store things in the new machine
                    List<TaskMDPTransition> currentlist = constructed.transitions.get(new IntegerPair(from, actionlabel));
                    if (currentlist == null) currentlist = new ArrayList<>();

//							System.out.println(from + "->" + to + " " + actionlabel + " " + currentlist);
//					System.out.println(from1 + "->" + to1 + ", " + from2 + "->" + to2 + " => " + from + "->" + to);

                    int trans = Integer.valueOf(to);
                    currentlist.add(new TaskMDPTransition(trans, prob));
                    constructed.transitions.put(new IntegerPair(from, actionlabel), currentlist);

                    List<String> dependencies2 = op2.symbolDependencies(from2);
                    List<String> dependencies = new ArrayList<String>(dependencies2);

                    constructed.symbolMap.put(from, dependencies);
                }
            }

            // from2 = 1, to2 = 1
            for (Map.Entry<IntegerPair, List<TaskMDPTransition>> e1 : this.transitions.entrySet()) {
                for (TaskMDPTransition trans1 : e1.getValue()) {
                    // get a pair of transitions from the old machines
                    int from2 = 1;
                    int from1 = e1.getKey().a;
                    int actionlabel = e1.getKey().b;
                    double prob = trans1.p;
                    int to2 = 1;
                    int to1 = Integer.valueOf(trans1.task);

                    // configure the combined transition  for the new machine
                    int from = and_combine(from1, from2, baseline);
                    int to = and_combine(to1, to2, baseline);

                    // Store things in the new machine
                    List<TaskMDPTransition> currentlist = constructed.transitions.get(new IntegerPair(from, actionlabel));
                    if (currentlist == null) currentlist = new ArrayList<>();

//							System.out.println(from + "->" + to + " " + actionlabel + " " + currentlist);
//					System.out.println(from1 + "->" + to1 + ", " + from2 + "->" + to2 + " => " + from + "->" + to);

                    int trans = Integer.valueOf( to);
                    currentlist.add(new TaskMDPTransition(trans, prob));
                    constructed.transitions.put(new IntegerPair(from, actionlabel), currentlist);

                    List<String> dependencies1 = this.symbolDependencies(from1);
                    List<String> dependencies = new ArrayList<String>(dependencies1);

                    constructed.symbolMap.put(from, dependencies);
                }
            }

            // Debugging
//			for (Map.Entry<Integer, List<String>> e : this.symbolMap.entrySet()) {
//				int state = e.getKey();
//				List<String> dependencies = e.getValue();
//				System.out.println(state + "/" + baseline + "+" + dependencies);
//				if (state > 1) {
//					constructed.symbolMap.put(state + baseline, dependencies);
//				}
//			}

            return constructed;
        }

        int and_combine(int node1, int node2, int baseline) {
            if ((node1 == 0) || (node2 == 0)) return 0;
            if ((node1 == 2) && (node2 == 2)) return 2;
            if ((node1 == 1) && (node2 == 1)) return 1;
            return node1*(baseline+2)+ node2;
        }

        // 1 needs to be true until 2 becomes true.
        // so we win if 2 is true. we lose if 1 is false.
        // restart if needed
        int until_combine(int node1, int node2, int baseline) {
            if (node2 == 1) return 1;
            if (node1 == 0) return 0;
            if (node1 == 1) node1 = 2; // great, keep making it true
            if (node2 == 0) node2 = 2; // fine, keep running since it hasn't become true yet.
            if ((node1 == 2) && (node2 == 2)) return 2;
            return node1*(baseline+2)+ node2;
        }
    }
}