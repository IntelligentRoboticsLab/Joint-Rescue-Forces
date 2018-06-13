/*******************************************************
 * Simple FireBrigade
 * 
 * @copyright Copyright 2012 Laboratorio de Tecnicas Inteligentes
 * @author Annibal B. M. da Silva
 * @author Luis G. Nardin
 * @author Jaime S. Sichman
 * @license BSD 3-Clause "New" or "Revised" License
 ********************************************************/
package matlab.generator.simple.agent.fire;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.EnumSet;
import java.util.List;
import rescuecore2.messages.Command;
import rescuecore2.standard.entities.Building;
import rescuecore2.standard.entities.FireBrigade;
import rescuecore2.standard.entities.Refuge;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.standard.entities.StandardEntityURN;
import rescuecore2.worldmodel.ChangeSet;
import rescuecore2.worldmodel.EntityID;
import matlab.generator.simple.agent.AbstractSimpleAgent;

public class SimpleFireBrigade extends AbstractSimpleAgent<FireBrigade> {
  
  private static final String MAX_WATER_KEY    = "fire.tank.maximum";
  
  private static final String MAX_DISTANCE_KEY = "fire.extinguish.max-distance";
  
  private static final String MAX_POWER_KEY    = "fire.extinguish.max-sum";
  
  private static enum State {
    MOVING_TO_REFUGEE,
    MOVING_TO_FIRE,
    EXTINGUISHING_FIRE,
    REFILLING,
    RANDOM_WALKING,
    TAKING_ALTERNATE_ROUTE
  };
  
  private int            maxWater;
  
  private int            maxDistance;
  
  private int            maxPower;
  
  private List<EntityID> refuges;
  
  private State          state;
  
  
  /**
   * Constructor
   * 
   * @param outputDir
   *          Output directory
   */
  public SimpleFireBrigade( String outputDir ) {
    this.outputDir = outputDir;
  }
  
  
  /**
   * Define the agent as a FireBrigade
   */
  @Override
  protected EnumSet<StandardEntityURN> getRequestedEntityURNsEnum() {
    return EnumSet.of( StandardEntityURN.FIRE_BRIGADE );
  }
  
  
  /**
   * Executed after connected to the simulator (1) initializes global variables
   * (2) extracts information about the refuges
   */
  @Override
  protected void postConnect() {
    super.postConnect();
    
    model.indexClass( StandardEntityURN.BUILDING, StandardEntityURN.REFUGE );
    maxWater = config.getIntValue( MAX_WATER_KEY );
    maxDistance = config.getIntValue( MAX_DISTANCE_KEY );
    maxPower = config.getIntValue( MAX_POWER_KEY );
    
    refuges = new ArrayList<EntityID>();
    List<Refuge> ref = getRefuges();
    
    for ( Refuge r : ref ) {
      refuges.add( r.getID() );
    }
    
    state = State.RANDOM_WALKING;
  }
  
  
  /**
   * Reason and make a decision for an action to perform in the environment
   * 
   * @param time
   *          Current timestep
   * @param changed
   *          Perception of changes in the environment
   * @param heard
   *          Voice messages heard
   */
  protected void think( int time, ChangeSet changed,
      Collection<Command> heard ) {
    // There is no need to stay inside a burning building, right?
    if ( location() instanceof Building ) {
      if ( ((Building) location()).isOnFire() ) {
        List<EntityID> path = randomWalk();
        
        sendMove( time, path );
        state = State.RANDOM_WALKING;
        return;
      }
    }
    
    if ( amIStuck() ) {
      List<EntityID> path = search.pathFinder( currentPosition,
          getBlockedRoads(), target );
      
      if ( path != null ) {
        state = State.TAKING_ALTERNATE_ROUTE;
      } else if ( state.equals( State.MOVING_TO_FIRE ) ) {
        List<EntityID> burning = getBurning();
        Collections.shuffle( burning, random );
        
        for ( EntityID next : burning ) {
          if ( !next.equals( target ) ) {
            target = next;
            path = search.breadthFirstSearch( currentPosition, target );
            break;
          }
        }
      } else {
        Collections.shuffle( refuges, random );
        
        for ( EntityID next : refuges ) {
          if ( !next.equals( target ) ) {
            target = next;
            path = search.breadthFirstSearch( currentPosition, target );
            break;
          }
        }
      }
    }
    // Am I at a refuge?
    if ( location() instanceof Refuge && me().isWaterDefined()
        && me().getWater() < maxWater ) {
      sendRest( time );
      state = State.REFILLING;
      return;
    }
    // Am I out of water?
    if ( me().isWaterDefined() && me().getWater() == 0 ) {
      List<EntityID> path = search.breadthFirstSearch( location().getID(),
          refuges );
      state = State.MOVING_TO_REFUGEE;
      
      if ( path == null ) {
        path = randomWalk();
        state = State.RANDOM_WALKING;
      }
      target = path.get( path.size() - 1 );
      sendMove( time, path );
      return;
    }
    
    List<EntityID> burning = getBurning();
    List<EntityID> path = new ArrayList<EntityID>();
    
    if ( !(burning.isEmpty()) ) {
      // Is there any burning building in range?
      for ( EntityID next : burning ) {
        if ( changed.getChangedEntities().contains( next )
            && model.getDistance( location().getID(), next ) < maxDistance ) {
          sendExtinguish( time, next, maxPower );
          target = next;
          state = State.EXTINGUISHING_FIRE;
          return;
        }
      }
      // No? Let us get to one, then.
      path = search.breadthFirstSearch( location().getID(), burning );
      if ( path != null ) {
        path.remove( path.size() - 1 );
        sendMove( time, path );
        state = State.MOVING_TO_FIRE;
        if ( !path.isEmpty() ) {
          target = path.get( path.size() - 1 );
        }
        return;
      }
    }
    
    path = randomWalk();
    sendMove( time, path );
    state = State.RANDOM_WALKING;
    return;
  }
  
  
  /**
   * Identify burning buildings
   * 
   * @return Buildings on fire
   */
  protected List<EntityID> getBurning() {
    List<EntityID> result = new ArrayList<EntityID>();
    Collection<StandardEntity> b = model
        .getEntitiesOfType( StandardEntityURN.BUILDING );
    
    for ( StandardEntity next : b ) {
      if ( next instanceof Building ) {
        if ( ((Building) next).isOnFire() ) {
          result.add( next.getID() );
        }
      }
    }
    
    return result;
  }
  
  
  /**
   * Identify if the agent is stuck
   * 
   * @return TRUE Agent is stuck, FALSE otherwise
   */
  private boolean amIStuck() {
    if ( lastPosition.getValue() == currentPosition.getValue()
        && (state.equals( State.MOVING_TO_FIRE )
            || state.equals( State.MOVING_TO_REFUGEE ))
        && getBlockedRoads().contains( location().getID() ) ) {
      return true;
    }
    return false;
  }
}