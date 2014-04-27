/*
Copyright (c) 2014, Jesús Martín Berlanga
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this 
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation 
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without 
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

package com.jme3.ai.agents.behaviours.npc.steering;

import com.jme3.ai.agents.Agent;
import com.jme3.math.Vector3f;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Spatial;
import java.util.Arrays;
import java.util.ArrayList;
import java.util.List;

/**
 * @author Jesús Martín Berlanga
 * @version 1.0
 */
public class SeparationBehaviour extends AbstractSteeringBehaviour{
    
    //List of the obstacles that we want to be separated
    private List<Agent> obstacles = new ArrayList<Agent>();
    
    public List<Agent> getObstacles(){return this.obstacles;}
    
    /**
     * @param agent To whom behaviour belongs.
     * @param initialObstacles Initializes a list with the obstacles from
     *   the agent want to be separated
     */
    public SeparationBehaviour(Agent agent, List<Agent> initialObstacles){
        super(agent);
        this.obstacles = initialObstacles;
    }
    
    /**
     * @param spatial active spatial during excecution of behaviour
     * @see SeparationBehaviour#SeparationBehaviour(com.jme3.ai.agents.Agent, com.jme3.ai.agents.Agent[]) 
     */
    public SeparationBehaviour(Agent agent, Agent[] initialObstacles, Spatial spatial) {
        super(agent, spatial);
        this.obstacles = Arrays.asList(initialObstacles);
    }
    
    @Override
    protected void controlRender(RenderManager rm, ViewPort vp) {
    }
    
    /** 
    * Separation steering behavior gives a character the ability to 
    * maintain a certain separation distance from others nearby. This 
    * can be used to prevent characters from crowding together.
    * 
    * For each nearby character, a repulsive force is computed by 
    * subtracting the positions of our character and the nearby character, 
    * normalizing, and then applying a 1/r weighting. (That is, the position 
    * offset vector is scaled by 1/r^2.) Note that 1/r is just a setting 
    * that has worked well, not a fundamental value. These repulsive forces 
    * for each nearby character are summed together to produce the overall 
    * steering force.
    * 
    * The supplied neighbours should only be the nearby neighbours in
    * the field of view of the character that is steering. It is good to
    * ignore anything behind the character.
    *
    * Original method by Brent Owens.
    * Adaptation to MonkeyBrains by Jesús Martín Berlanga
    *
    * @see AbstractSteeringBehaviour#calculateSteering() 
    */
    protected Vector3f calculateSteering(){
        //Propities whom behaviour belongs.
        Vector3f agentLocation = super.agent.getLocalTranslation();
        
        Vector3f steering = new Vector3f();
        
        for (Agent oAgent : this.obstacles) {
                Vector3f loc = oAgent.getLocalTranslation().subtract(agentLocation);
                float len2 = loc.lengthSquared();
                loc.normalizeLocal();
                steering.addLocal(loc.negate().mult(1f/((float) Math.pow(len2, 2))));
        }
        
       
        System.out.println(steering.length());
        return steering;
    }
    
    /**
     * Calculate new velocity for agent based on calculated steering behaviour.
     *
     * @return velocity vector
     * 
     * @author Tihomir Radosavljević
     */
    protected Vector3f calculateNewVelocity() {
        Vector3f steering = calculateSteering();
        if (steering.length() > agent.getMaxForce()) {
            steering = steering.normalize().mult(agent.getMaxForce());
        }
        agent.setAcceleration(steering.mult(1 / agentTotalMass()));
        velocity = velocity.add(agent.getAcceleration());
        if (velocity.length() > agent.getMaxMoveSpeed()) {
            velocity = velocity.normalize().mult(agent.getMaxMoveSpeed());
        }
        return velocity;
    }
}
