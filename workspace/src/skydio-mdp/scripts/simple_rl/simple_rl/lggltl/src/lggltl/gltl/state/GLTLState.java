package lggltl.gltl.state;

import burlap.mdp.core.oo.state.MutableOOState;
import burlap.mdp.core.oo.state.OOState;
import burlap.mdp.core.oo.state.OOStateUtilities;
import burlap.mdp.core.oo.state.ObjectInstance;
import burlap.mdp.core.state.MutableState;
import burlap.mdp.core.state.State;
import lggltl.gltl.GLTLCompiler;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by dilip and nakul on 10/13/17.
 * This is a wrapper state around the base state that holds on to the
 * GLTL spec attribute along with the environment state.
 */
public class GLTLState implements MutableOOState {

    public int spec;
    public String StateType = "type";

    public OOState envState;

    public GLTLState(OOState envState){
        this.envState = envState;
        this.spec = 2;
    }

    public GLTLState(OOState envState, int spec){
        this.envState = envState;
        this.spec = spec;
    }

    public GLTLState(){}

    public State getBaseState(){
        return envState;
    }

    @Override
    public MutableOOState addObject(ObjectInstance objectInstance) {
        throw new RuntimeException("Cannot add objects to GLTLState.");
    }

    @Override
    public MutableOOState removeObject(String s) {
        throw new RuntimeException("Cannot remove objects from GLTLState.");
    }

    @Override
    public MutableOOState renameObject(String s, String s1) {
        throw new RuntimeException("Cannot rename objects from GLTLState.");
    }

    @Override
    public int numObjects() {
        //TODO: one object is the spec and another is the base state?? Do not think we will need this method!
        return 1+envState.numObjects();
    }

    @Override
    public ObjectInstance object(String s) {
        if(s.equals(GLTLCompiler.CLASSSPEC)){
            return new PseudoObject(GLTLCompiler.CLASSSPEC, this.spec);

        }
        else return envState.object(s);
    }

    @Override
    public List<ObjectInstance> objects() {
        List<ObjectInstance> objects = new ArrayList<>();
        objects = this.envState.objects();
        objects.add(new PseudoObject(GLTLCompiler.CLASSSPEC, this.spec));
        return objects;
    }

    @Override
    public List<ObjectInstance> objectsOfClass(String s) {
        if(s.equals(GLTLCompiler.CLASSSPEC)){
            List<ObjectInstance> objects = new ArrayList<>();
            objects.add(new PseudoObject(GLTLCompiler.CLASSSPEC, this.spec));
            return objects;
        }
        else return envState.objectsOfClass(s);
    }

    @Override
    public MutableState set(Object o, Object o1) {
        throw new RuntimeException("Cannot set objects in GLTLState.");
    }

    @Override
    public List<Object> variableKeys() {
        throw new RuntimeException("Cannot fetch objects from GLTLState.");
    }

    @Override
    public Object get(Object o) {
        throw new RuntimeException("Cannot fetch objects from GLTLState.");
    }

    @Override
    public GLTLState copy() {
        return new GLTLState((OOState)envState.copy(), Integer.valueOf(spec));
    }

    @Override
    public boolean equals(Object o){
        // self check
        if (this == o)
            return true;
        // null check
        if (o == null)
            return false;
        // type check and cast
        if (getClass() != o.getClass())
            return false;
        GLTLState gs = (GLTLState) o;
        // field comparison
        return this.spec==gs.spec && this.envState.equals(gs.envState);
    }

    @Override
    public int hashCode(){
        return (this.spec +"").hashCode() + this.envState.hashCode();
    }

    @Override
    public String toString() {
        return OOStateUtilities.ooStateToString(this);
    }

}
