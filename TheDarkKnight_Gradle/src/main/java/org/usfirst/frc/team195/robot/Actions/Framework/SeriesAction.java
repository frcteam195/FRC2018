package org.usfirst.frc.team195.robot.Actions.Framework;

import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;

import java.util.ArrayList;
import java.util.List;

/**
 * Executes one action at a time. Useful as a member of {@link ParallelAction}
 */
public class SeriesAction implements Action {

    private Action mCurAction;
    private final ArrayList<Action> mRemainingActions;

    public SeriesAction(List<Action> actions) {
        mRemainingActions = new ArrayList<>(actions.size());

        for (Action action : actions) {
            mRemainingActions.add(action);
        }

        mCurAction = null;
    }

    @Override
    public boolean isFinished() {
        ConsoleReporter.report("Series queue empty: " + mRemainingActions.isEmpty());
        ConsoleReporter.report("Series inner action null: " + (mCurAction == null));
        return mRemainingActions.isEmpty() && mCurAction == null;
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        if (mCurAction == null) {
            if (mRemainingActions.isEmpty()) {
                return;
            }

            mCurAction = mRemainingActions.remove(0);
            mCurAction.start();
        }

        mCurAction.update();

        ConsoleReporter.report(mCurAction.getClass().getName());

        if (mCurAction.isFinished()) {
            mCurAction.done();
            mCurAction = null;
        }
    }

    @Override
    public void done() {
    }
}
