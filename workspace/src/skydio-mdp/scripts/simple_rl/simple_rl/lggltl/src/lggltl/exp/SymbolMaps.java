package lggltl.exp;

import burlap.mdp.core.oo.propositional.GroundedProp;
import burlap.mdp.core.oo.propositional.PropositionalFunction;
import lggltl.cleanup.CleanupDomain;

import java.util.HashMap;
import java.util.Map;

import static lggltl.cleanup.CleanupDomain.*;

/**
 * Created by dilip on 10/31/17.
 */
public class SymbolMaps {

    private static final PropositionalFunction A2R = new CleanupDomain.PF_InRegion(PF_AGENT_IN_ROOM, new String[]{CLASS_AGENT, CLASS_ROOM}, false);
    private static final PropositionalFunction B2R = new CleanupDomain.PF_InRegion(PF_BLOCK_IN_ROOM, new String[]{CLASS_BLOCK, CLASS_ROOM}, false);

    /** CS prefix denotes formulas applicable in the CleanupDomain.getClassicState environment **/
    /** FR prefix denotes formulas applicable in the CleanupDomain.getState environment (with four rooms) **/

    protected static final Map<String, GroundedProp> CS_SYMBOL_MAP = new HashMap<>();

    protected static final Map<String, GroundedProp> FR_SYMBOL_MAP = new HashMap<>();

    static {
        CS_SYMBOL_MAP.put("R", new GroundedProp(A2R, new String[]{"agent0", "room0"}));
        CS_SYMBOL_MAP.put("C", new GroundedProp(A2R, new String[]{"agent0", "room1"}));
        CS_SYMBOL_MAP.put("B", new GroundedProp(A2R, new String[]{"agent0", "room2"}));

        FR_SYMBOL_MAP.put("R", new GroundedProp(A2R, new String[]{"agent0", "room0"}));
        FR_SYMBOL_MAP.put("B", new GroundedProp(A2R, new String[]{"agent0", "room1"}));
        FR_SYMBOL_MAP.put("C", new GroundedProp(A2R, new String[]{"agent0", "room2"}));
        FR_SYMBOL_MAP.put("Y", new GroundedProp(A2R, new String[]{"agent0", "room3"}));

        FR_SYMBOL_MAP.put("X", new GroundedProp(B2R, new String[]{"block0", "room2"}));
    }
}
