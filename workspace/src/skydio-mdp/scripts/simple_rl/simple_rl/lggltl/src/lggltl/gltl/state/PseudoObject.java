package lggltl.gltl.state;


import burlap.mdp.core.oo.state.OOStateUtilities;
import burlap.mdp.core.oo.state.ObjectInstance;
import burlap.mdp.core.state.annotations.DeepCopyState;

import java.util.Arrays;
import java.util.List;
import static lggltl.gltl.GLTLCompiler.*;


/**
 * Created by ngopalan .
 */
@DeepCopyState
public class PseudoObject implements ObjectInstance {

    public String name;
    public int x;
    private final static List<Object> keys = Arrays.<Object>asList(ATTSPEC);

    public PseudoObject(String name, int x){
        this.name = name;
        this.x= x;
    }

    public PseudoObject(){}


    @Override
    public String className() {
        return CLASSSPEC;
    }

    @Override
    public String name() {
        return name;
    }

    @Override
    public PseudoObject copyWithName(String objectName) {
        return new PseudoObject(objectName, x);
    }

    @Override
    public List<Object> variableKeys() {
        return keys;
    }

    @Override
    public Object get(Object variableKey) {

        if(!(variableKey instanceof String)){
            throw new RuntimeException("PseudoObject variable key must be a string");
        }

        String key = (String)variableKey;
        if(key.equals(ATTSPEC)){
            return x;
        }

        throw new RuntimeException("Unknown key for PseudoObject: " + key);
    }

    @Override
    public PseudoObject copy() {
        return new PseudoObject(name,x);
    }

    @Override
    public boolean equals(Object o) {
        // self check
        if (this == o)
            return true;
        // null check
        if (o == null)
            return false;
        // type check and cast
        if (getClass() != o.getClass())
            return false;
        PseudoObject that = (PseudoObject) o;
        // field comparison
        return (this.x==that.x)
                && (this.name == that.name());
    }


    @Override
    public String toString() {
        return OOStateUtilities.objectInstanceToString(this);
    }
}
