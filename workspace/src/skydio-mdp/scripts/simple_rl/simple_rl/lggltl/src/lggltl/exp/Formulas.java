package lggltl.exp;

import java.util.HashMap;
import java.util.Map;

/**
 * Created by dilip on 10/31/17.
 */
public class Formulas {

    public static final String EVENT_RED = "F4R";
    public static final String EVENT_GREEN = "F4C";
    public static final String EVENT_BLUE = "F4B";
    public static final String EVENT_YELLOW = "F4Y";

    public static final String ALW_EVENT_GREEN_AND_EVENT_BLUE = "G4F4&CF1B"; //Always eventually (Green and eventually Blue)
    public static final String ALW_EVENT_RED_AND_EVENT_BLUE = "G4F4&RF4B"; //Always eventually (Red and eventually Blue)
    public static final String ALW_EVENT_BLUE_AND_EVENT_YELLOW = "G4F4&BF4Y"; //Always eventually (Blue and eventually Yellow)

    public static final String EVENT_BLOCK2GREEN = "F4X";
    public static final String EVENT_BLOCK2GREEN_AND_NEVER_BLUE = "&F4XG4!B";
    public static final String EVENT_BLOCK2GREEN_AND_NEVER_YELLOW = "&F4XG4!Y";

    public static final String EVENT_GREEN_NEVER_BLUE = "U4!BC";
    //    public static final String EVENT_BLUE_NEVER_GREEN = "&F4BG4!C";
    public static final String EVENT_BLUE_NEVER_GREEN = "U4!CB";



    public static final String BAXTER_ALW_SCAN_UNTIL_BLOCK ="G2&U2S!AF2A";
    public static final String BAXTER_ALW_SCAN_UNTIL_NON_RED_BLOCK ="G2&U2S!RF2R";
    public static final String BAXTER_ALW_SCAN_UNTIL_NON_BLUE_BLOCK ="G2&U2S!BF2B";
    public static final String BAXTER_ALW_SCAN_UNTIL_NON_GREEN_BLOCK ="G2&U2S!CF2C";


    public static final String ROTATE_FOUR_ROOMS = "G4F4&BF4&CF4&YF4R";




    public static Map<String,String> generateFormulaeMap(){
        Map<String,String> mapping_formulae = new HashMap<>();
        mapping_formulae.put("F & Y F B","F4&YF4B");//F4&YF4B
        mapping_formulae.put("F & Y F C","F4&YF4C");
        mapping_formulae.put("F R","F4R");
        mapping_formulae.put("F & | C Y F R","F4&|CYF4R");
        mapping_formulae.put("F Y","F4Y");
        mapping_formulae.put("F C","F4C");
        mapping_formulae.put("F B","U4TB");
        mapping_formulae.put("F & | R C F B","F4&|RCF4B");
        mapping_formulae.put("F & | R Y F B","F4&|RYF4B");
        mapping_formulae.put("F & | R Y F C","F4&|RYF4C");
        mapping_formulae.put("& F Y G ! R","U4!RY");
        mapping_formulae.put("F & | C Y F B","F4&|CYF4B");
        mapping_formulae.put("F & | R B F Y","F4&|RBF4Y");
        mapping_formulae.put("F & | R B F C","F4&|RBF4C");
        mapping_formulae.put("& F Y G ! B","U4!BY");
        mapping_formulae.put("& F Y G ! C","U4!CY");
        mapping_formulae.put("& F R G ! B","U4!BR");
        mapping_formulae.put("& F B G ! Y","U4!YB");
        mapping_formulae.put("& F R G ! Y","U4!YR");
        mapping_formulae.put("F & C F Y","F4&CF4Y");
        mapping_formulae.put("F & | Y B F C","F4&|YBF4C");
        mapping_formulae.put("F & B F C","F4&BF4C");
        mapping_formulae.put("F & B F Y","F4&BF4Y");
        mapping_formulae.put("F & C F R","F4&CF4R");
        mapping_formulae.put("F & R F Y","F4&RF4Y");
        mapping_formulae.put("F & | C R F B","F4&|CRF4B");
        mapping_formulae.put("F & C F B","U4F4CB");//F4&CF4B
        mapping_formulae.put("F & B F R","F4&BF4R");
        mapping_formulae.put("& F B G ! C","U4!CB");
        mapping_formulae.put("F & R F X","F4&RF4X");
        mapping_formulae.put("& F C G ! Y","U4!YC");
        mapping_formulae.put("F & R F Z","F4&RF4Z");
        mapping_formulae.put("& F C G ! R","U4!RC");
        mapping_formulae.put("F & | B Y F C","F4&|BYF4C");
        mapping_formulae.put("& F B G ! R","U4!RB");
        mapping_formulae.put("& F R G ! C","U4!CR");
        mapping_formulae.put("& F C G ! B","U4!BC");
        mapping_formulae.put("F & R F C","F4&RF4C");
        mapping_formulae.put("F & R F B","F4&RF4B");


//        Map<String,String> mapping_formulae = new HashMap<>();
//        mapping_formulae.put("F & Y F B","F4&YF4B");
//        mapping_formulae.put("F & Y F C","F4&YF4C");
//        mapping_formulae.put("F R","F4R");
//        mapping_formulae.put("F & | C Y F R","F4&|CYF4R");
//        mapping_formulae.put("F Y","F4Y");
//        mapping_formulae.put("F C","F4C");
//        mapping_formulae.put("F B","F4B");
//        mapping_formulae.put("F & | R C F B","F4&|RCF4B");
//        mapping_formulae.put("F & | R Y F B","F4&|RYF4B");
//        mapping_formulae.put("F & | R Y F C","F4&|RYF4C");
//        mapping_formulae.put("& F Y G ! R","&F4YG4!R");
//        mapping_formulae.put("F & | C Y F B","F4&|CYF4B");
//        mapping_formulae.put("F & | R B F Y","F4&|RBF4Y");
//        mapping_formulae.put("F & | R B F C","F4&|RBF4C");
//        mapping_formulae.put("& F Y G ! B","&F4YG4!B");
//        mapping_formulae.put("& F Y G ! C","&F4YG4!C");
//        mapping_formulae.put("& F R G ! B","&F4RG4!B");
//        mapping_formulae.put("& F B G ! Y","&F4BG4!Y");
//        mapping_formulae.put("& F R G ! Y","&F4RG4!Y");
//        mapping_formulae.put("F & C F Y","F4&CF4Y");
//        mapping_formulae.put("F & | Y B F C","F4&|YBF4C");
//        mapping_formulae.put("F & B F C","F4&BF4C");
//        mapping_formulae.put("F & B F Y","F4&BF4Y");
//        mapping_formulae.put("F & C F R","F4&CF4R");
//        mapping_formulae.put("F & R F Y","F4&RF4Y");
//        mapping_formulae.put("F & | C R F B","F4&|CRF4B");
//        mapping_formulae.put("F & C F B","F4&CF4B");
//        mapping_formulae.put("F & B F R","F4&BF4R");
//        mapping_formulae.put("& F B G ! C","&F4BG4!C");
//        mapping_formulae.put("F & R F X","F4&RF4X");
//        mapping_formulae.put("& F C G ! Y","&F4CG4!Y");
//        mapping_formulae.put("F & R F Z","F4&RF4Z");
//        mapping_formulae.put("& F C G ! R","&F4CG4!R");
//        mapping_formulae.put("F & | B Y F C","F4&|BYF4C");
//        mapping_formulae.put("& F B G ! R","&F4BG4!R");
//        mapping_formulae.put("& F R G ! C","&F4RG4!C");
//        mapping_formulae.put("& F C G ! B","&F4CG4!B");
//        mapping_formulae.put("F & R F C","F4&RF4C");
//        mapping_formulae.put("F & R F B","F4&RF4B");

<<<<<<< HEAD

        return mapping_formulae;
    }
=======
    protected static final String ALW_EVENT_GREEN_AND_EVENT_BLUE = "G4F4&CF4B"; //Always eventually (Green and eventually Blue)
    protected static final String ALW_EVENT_RED_AND_EVENT_BLUE = "G4F4&RF4B"; //Always eventually (Red and eventually Blue)
    protected static final String ALW_EVENT_BLUE_AND_EVENT_YELLOW = "G4F4&BF4Y"; //Always eventually (Blue and eventually Yellow)

    protected static final String EVENT_BLOCK2GREEN = "F4X";
    protected static final String EVENT_BLOCK2GREEN_AND_NEVER_BLUE = "&F4XG4!B";
    protected static final String EVENT_BLOCK2GREEN_AND_NEVER_BLUE_UNTIL_BLOCK2GREEN = "&F4XU4G4!BX";
    protected static final String EVENT_BLOCK2GREEN_AND_NEVER_YELLOW = "&F4XG4!Y";
>>>>>>> 20ba64d71fd0f8552b07dedaa4b2a50641262fc4

}
