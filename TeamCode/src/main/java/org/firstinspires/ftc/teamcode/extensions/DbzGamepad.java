package org.firstinspires.ftc.teamcode.extensions;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;
import org.firstinspires.ftc.teamcode.utils.LogDbz;


/**
 * Created by Matthew on 9/7/2017.
 */

public class DbzGamepad {
    private static final String TAG = DbzGamepad.class.getName();
    private Gamepad gamepad;

    public DbzGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
        update();
    }

    private DpadEventCallback dpadEventCallback = null;

    public DpadEventCallback getDpadEventCallback() {
        return dpadEventCallback;
    }

    public void setDpadEventCallback(DpadEventCallback dpadEventCallback) {
        this.dpadEventCallback = dpadEventCallback;
    }

    /**
     * left analog stick horizontal axis
     */
    public float left_stick_x;

    /**
     * left analog stick vertical axis
     * <p>
     * the sdk implements this reversed for some reason.  so we'll unreverse it here
     */
    public float left_stick_y;

    /**
     * right analog stick horizontal axis
     */
    public float right_stick_x;

    /**
     * right analog stick vertical axis
     */
    public float right_stick_y;

    /**
     * dpad up
     */
    public boolean dpad_up;

    /**
     * dpad down
     */
    public boolean dpad_down;

    /**
     * dpad left
     */
    public boolean dpad_left;

    /**
     * dpad right
     */
    public boolean dpad_right;

    /**
     * button a
     */
    public boolean a;

    /**
     * button b
     */
    public boolean b;

    /**
     * button x
     */
    public boolean x;

    /**
     * button y
     */
    public boolean y;
    /**
     * button guide - often the large button in the middle of the controller. The OS may
     * capture this button before it is sent to the app; in which case you'll never
     * receive it.
     */
    public boolean guide;

    /**
     * button startRecordingStalling
     */
    public boolean start;

    /**
     * button back
     */
    public boolean back;

    /**
     * button left bumper
     */
    public boolean left_bumper;

    /**
     * button right bumper
     */
    public boolean right_bumper;

    /**
     * left stick button
     */
    public boolean left_stick_button;

    /**
     * right stick button
     */
    public boolean right_stick_button;

    /**
     * left trigger
     */
    public float left_trigger;

    /**
     * right trigger
     */
    public float right_trigger;

    /**
     * true if any of the dpad buttons are pressed
     */
    public boolean dpad;

    //
    public GamepadUser getUser() {
        return gamepad.getUser();
    }


    //
    public void setUser(GamepadUser user) {
        gamepad.setUser(user);
    }


    public void setGamepadId(int id) {
        gamepad.setGamepadId(id);
    }

    public int getGamepadId() {
        return gamepad.getGamepadId();
    }

    /**
     * Relative timestamp of the last time an event was detected
     */
    public long timestamp;

    /**
     * Sets the time at which this Gamepad last changed its state,
     * in the {@link SystemClock#uptimeMillis} time base.
     */
    public void setTimestamp(long timestamp) {
        gamepad.setTimestamp(timestamp);
    }

    /**
     * Refreshes the Gamepad's timestamp to be the current time.
     */
    public void refreshTimestamp() {
        setTimestamp(SystemClock.uptimeMillis());
    }

    /**
     * Set the joystick deadzone. Must be between 0 and 1.
     *
     * @param deadzone amount of joystick deadzone
     */


    /**
     * Are all analog sticks and triggers in their rest position?
     *
     * @return true if all analog sticks and triggers are at rest; otherwise false
     */
    public boolean atRest() {
        return gamepad.atRest();
    }

    public String toString() {
        return gamepad.toString();
    }

    public void update(){
        boolean shouldRunCallback = false;
        if(dpad_up != gamepad.dpad_up || dpad_down != gamepad.dpad_down ||
                dpad_left != gamepad.dpad_left || dpad_right != gamepad.dpad_right){
            LogDbz.v(TAG,"Dpad update event triggered");
            if(dpadEventCallback != null){
                shouldRunCallback = true;
            }
        }

        left_stick_x = gamepad.left_stick_x;
        left_stick_y = -gamepad.left_stick_y;
        right_stick_x = gamepad.right_stick_x;
        right_stick_y = gamepad.right_stick_y;

        dpad_up = gamepad.dpad_up;
        dpad_down = gamepad.dpad_down;
        dpad_left = gamepad.dpad_left;
        dpad_right = gamepad.dpad_right;
        dpad = gamepad.dpad_up || gamepad.dpad_down || gamepad.dpad_left || gamepad.dpad_right;

        a = gamepad.a;
        b = gamepad.b;
        x = gamepad.x;
        y = gamepad.y;

        guide = gamepad.guide;
        start = gamepad.start;
        back = gamepad.back;

        left_bumper = gamepad.left_bumper;
        right_bumper = gamepad.right_bumper;

        left_stick_button = gamepad.left_stick_button;
        right_stick_button = gamepad.right_stick_button;

        left_trigger = gamepad.left_trigger;
        right_trigger = gamepad.right_trigger;

        timestamp = gamepad.timestamp;

        if(shouldRunCallback)
            dpadEventCallback.process(this);
    }


    public interface DpadEventCallback  {
        void process(DbzGamepad gamepad);
    }
}
