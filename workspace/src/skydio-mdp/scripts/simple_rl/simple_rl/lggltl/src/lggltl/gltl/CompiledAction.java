package lggltl.gltl;

import burlap.mdp.core.action.Action;
import lggltl.gltl.GLTLCompiler;

public class CompiledAction implements Action {
    Action srcAction;
    String compiledActionName = "compiledAction";
    GLTLCompiler.CompiledActionType at;

    public CompiledAction(Action srcAction, GLTLCompiler.CompiledActionType at){
        this.srcAction = srcAction;
        this.at = at;
    }

    @Override
    public String actionName() {
        return compiledActionName+"_"+ srcAction.actionName();
    }

    @Override
    public Action copy() {
        return null;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        CompiledAction that = (CompiledAction) o;
        return this.srcAction.equals((that).srcAction);
    }

    public Action getSourceAction(){
        return this.srcAction;
    }

    public GLTLCompiler.CompiledActionType getCompiledActionType(){
        return this.at;
    }

    @Override
    public int hashCode() {
        return compiledActionName.hashCode() + srcAction.hashCode();
    }

    @Override
    public String toString() {
        return this.actionName() + "_" + this.srcAction.actionName();
    }
}
