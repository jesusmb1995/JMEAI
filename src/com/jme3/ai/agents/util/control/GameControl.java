package com.jme3.ai.agents.util.control;

import com.jme3.ai.agents.Agent;
import com.jme3.ai.agents.util.GameObject;
import com.jme3.math.Vector3f;

/**
 * Base interface for game controls.
 *
 * @author Tihomir Radosavljević
 * @version 0.1
 */
public interface GameControl {

    /**
     * Add all inputManagerMapping that will player use.
     */
    public void loadInputManagerMapping();
    /**
     * Method for marking the end of game. Should also set over to true.
     * @return 
     */
    public boolean finish();
    /**
     * Calculating if the agent won the game.
     * @param agent
     * @return 
     */
    public boolean win(Agent agent);
    /**
     * Restarting all game parameters.
     */
    public void restart();
   
    /**
     * Method for creating objects in given area.
     * @param gameObject object that should be created
     * @param area where object will be created
     */
    public void spawn(GameObject gameObject, Vector3f... area);
}
