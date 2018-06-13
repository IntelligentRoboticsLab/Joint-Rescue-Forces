/*******************************************************
 * Simple Ambulance Team
 * 
 * @copyright Copyright 2012 Laboratorio de Tecnicas Inteligentes
 * @author Annibal B. M. da Silva
 * @author Luis G. Nardin
 * @author Jaime S. Sichman
 * @license BSD 3-Clause "New" or "Revised" License
 ********************************************************/
package matlab.generator.simple.agent.ambulance;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import rescuecore2.log.Logger;
import rescuecore2.messages.Command;
import rescuecore2.standard.entities.AmbulanceTeam;
import rescuecore2.standard.entities.Building;
import rescuecore2.standard.entities.Civilian;
import rescuecore2.standard.entities.Human;
import rescuecore2.standard.entities.Refuge;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.standard.entities.StandardEntityURN;
import rescuecore2.worldmodel.ChangeSet;
import rescuecore2.worldmodel.EntityID;
import matlab.generator.simple.agent.AbstractSimpleAgent;

public class SimpleAmbulanceTeam extends AbstractSimpleAgent<AmbulanceTeam> {
  
  private static enum State {
    CARRYING_CIVILIAN,
    PATROLLING,
    RESCUEING,
    RANDOM_WALKING,
    DEAD,
    BURIED,
    TAKING_ALTERNATE_ROUTE
  };
  
  private List<EntityID>                     buildingsToCheck;
  
  private List<EntityID>                     refuges;
  
  private State                              state;
  
  private EntityID                           victimId;
  
  private Map<Integer, Map<String, Integer>> victimInfo;
  
  
  /**
   * Constructor
   * 
   * @param outputDir
   *          Output directory
   */
  public SimpleAmbulanceTeam( String outputDir ) {
    this.outputDir = outputDir;
  }
  
  
  /**
   * Define the agent as an Ambulance Team
   */
  @Override
  protected EnumSet<StandardEntityURN> getRequestedEntityURNsEnum() {
    return EnumSet.of( StandardEntityURN.AMBULANCE_TEAM );
  }
  
  
  /**
   * Executed after connected to the simulator (1) initializes global variables
   * (2) extracts information about the refuges and buildings to check
   */
  @Override
  protected void postConnect() {
    super.postConnect();
    
    victimInfo = new HashMap<Integer, Map<String, Integer>>();
    
    refuges = new ArrayList<EntityID>();
    Collection<Refuge> ref = getRefuges();
    
    for ( Refuge next : ref ) {
      refuges.add( next.getID() );
    }
    
    buildingsToCheck = new ArrayList<EntityID>();
    
    for ( EntityID next : buildingIDs ) {
      if ( !refuges.contains( next ) ) {
        buildingsToCheck.add( next );
      }
    }
    
    state = State.RANDOM_WALKING;
    victimId = null;
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
  @Override
  protected void think( int time, ChangeSet changed,
      Collection<Command> heard ) {
    lastPosition = currentPosition;
    currentPosition = me().getPosition();
    
    if ( me().getHP() == 0 ) {
      state = State.DEAD;
      return;
    }
    
    if ( me().getBuriedness() != 0 ) {
      state = State.BURIED;
      return;
    }
    
    if ( amIStuck() ) {
      List<EntityID> path = search.pathFinder( currentPosition,
          getBlockedRoads(), target );
      
      if ( path != null ) {
        state = State.TAKING_ALTERNATE_ROUTE;
      } else if ( state.equals( State.PATROLLING ) ) {
        Collections.shuffle( buildingsToCheck, random );
        
        for ( EntityID next : buildingsToCheck ) {
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
      
      if ( path != null ) {
        sendMove( time, path );
      }
      
      return;
    }
    
    // Am I carrying a civilian?
    if ( someoneOnBoard() ) {
      state = State.CARRYING_CIVILIAN;
      // Am I at a refuge?
      if ( (refuges.contains( location().getID() ))
          && (model.getEntity( victimId ) != null) ) {
        
        System.out.println( "UNLOADING" );
        
        Map<String, Integer> info = this.victimInfo
            .get( this.victimId.getValue() );
        
        info.put( "eTime", time );
        info.put( "eDist", 0 );
        info.put( "eHP", ((Human) model.getEntity( victimId )).getHP() );
        info.put( "eDamage",
            ((Human) model.getEntity( victimId )).getDamage() );
        
        this.victimInfo.put( this.victimId.getValue(), info );
        
        sendUnload( time );
        this.victimId = null;
        return;
      } else if ( (model.getEntity( victimId ) != null)
          && (((Human) model.getEntity( victimId )).isHPDefined())
          && (((Human) model.getEntity( victimId )).getHP() <= 0) ) {
        
        System.out.println( "UNLOADING" );
        
        List<EntityID> path = search.breadthFirstSearch( me().getPosition(),
            refuges );
        
        int distance = -1;
        if ( path != null ) {
          distance = model.getDistance( location().getID(),
              path.get( path.size() - 1 ) );
        }
        
        Human victim = (Human) model.getEntity( victimId );
        
        Map<String, Integer> info = this.victimInfo
            .get( this.victimId.getValue() );
        
        info.put( "eTime", time );
        info.put( "eDist", distance );
        info.put( "eHP", victim.getHP() );
        info.put( "eDamage", victim.getDamage() );
        
        this.victimInfo.put( this.victimId.getValue(), info );
        
        sendUnload( time );
        this.victimId = null;
        return;
      }
      
      // No? I need to get to one, then.
      List<EntityID> path = search.breadthFirstSearch( me().getPosition(),
          refuges );
      
      if ( path == null ) {
        path = randomWalk();
        state = State.RANDOM_WALKING;
      }
      target = path.get( path.size() - 1 );
      sendMove( time, path );
      return;
    }
    // Am I inside a building that is not a refuge?
    if ( location() instanceof Building
        && !refuges.contains( location().getID() ) ) {
      // I will check this building now.
      buildingsToCheck.remove( location().getID() );
      
      /*
       * Elaborate a list of the possible victims inside the building, in the
       * following priority order: civilians, ambulances, fire brigades and
       * police forces.
       */
      List<EntityID> possibleVictims = new ArrayList<EntityID>();
      
      possibleVictims.addAll(
          getVisibleEntitiesOfType( StandardEntityURN.CIVILIAN, changed ) );
      possibleVictims.addAll( getVisibleEntitiesOfType(
          StandardEntityURN.AMBULANCE_TEAM, changed ) );
      possibleVictims.addAll(
          getVisibleEntitiesOfType( StandardEntityURN.FIRE_BRIGADE, changed ) );
      possibleVictims.addAll(
          getVisibleEntitiesOfType( StandardEntityURN.POLICE_FORCE, changed ) );
      
      // Is there a victim in this building?
      if ( !possibleVictims.isEmpty() ) {
        for ( EntityID next : possibleVictims ) {
          Human victim = (Human) model.getEntity( next );
          if ( victim.isHPDefined() ) {
            // Is (s)he alive?
            if ( victim.getHP() != 0 ) {
              if ( victim.getPosition().equals( me().getPosition() ) ) {
                // Is (s)he burried?
                if ( victim.getBuriedness() != 0 ) {
                  
                  if ( state != State.RESCUEING ) {
                    
                    System.out.println( "RESCUEING" );
                    
                    victimId = victim.getID();
                    
                    List<EntityID> pathVictim = search.breadthFirstSearch(
                        me().getPosition(), victim.getPosition() );
                    
                    List<EntityID> pathRefuge = search
                        .breadthFirstSearch( victim.getPosition(), refuges );
                    
                    int distance = -1;
                    if ( (pathVictim != null) && (pathRefuge != null) ) {
                      distance = model.getDistance( location().getID(),
                          pathVictim.get( pathVictim.size() - 1 ) )
                          + model.getDistance( victim.getPosition(),
                              pathRefuge.get( pathRefuge.size() - 1 ) );
                      
                    }
                    
                    Map<String, Integer> info = new HashMap<String, Integer>();
                    info.put( "sTime", time );
                    info.put( "sDist", distance );
                    info.put( "sHP", victim.getHP() );
                    info.put( "sDamage", victim.getDamage() );
                    info.put( "sBuriedness", victim.getBuriedness() );
                    
                    this.victimInfo.put( victim.getID().getValue(), info );
                  }
                  
                  sendRescue( time, victim.getID() );
                  state = State.RESCUEING;
                  return;
                } else if ( victim instanceof Civilian ) {
                  sendLoad( time, victim.getID() );
                  return;
                }
              }
            }
          }
        }
      }
    }
    // Nothing to do here. Moving on.
    Set<EntityID> safeBuildings = getSafeBuildings( changed );
    List<EntityID> path;
    
    for ( EntityID next : buildingsToCheck ) {
      // I need to check if it's safe to go inside this building.
      if ( safeBuildings.contains( next ) ) {
        path = search.breadthFirstSearch( me().getPosition(), next );
        if ( path != null ) {
          target = path.get( path.size() - 1 );
          sendMove( time, path );
          state = State.PATROLLING;
          return;
        }
      }
    }
    /*
     * There are no safe buildings nearby. Going for a farther one (although I'm
     * not really sure it's safe there)
     */
    for ( EntityID next : buildingsToCheck ) {
      if ( !(((Building) model.getEntity( next )).isOnFire()) ) {
        path = search.breadthFirstSearch( me().getPosition(), next );
        if ( path != null ) {
          target = path.get( path.size() - 1 );
          sendMove( time, path );
          state = State.PATROLLING;
          return;
        }
      }
    }
    path = randomWalk();
    sendMove( time, path );
    state = State.RANDOM_WALKING;
    return;
  }
  
  
  /**
   * Identify if someone is on board of an Ambulance Team
   * 
   * @return TRUE Someone is on board, FALSE otherwise
   */
  private boolean someoneOnBoard() {
    for ( StandardEntity next : model
        .getEntitiesOfType( StandardEntityURN.CIVILIAN ) ) {
      if ( ((Human) next).getPosition().equals( getID() ) ) {
        Logger.debug( next + " is on board" );
        return true;
      }
    }
    return false;
  }
  
  
  /**
   * Identify if the agent is stuck
   * 
   * @return TRUE Agent is stuck, FALSE otherwise
   */
  private boolean amIStuck() {
    if ( lastPosition.getValue() == currentPosition.getValue()
        && (state.equals( State.CARRYING_CIVILIAN )
            || state.equals( State.PATROLLING ))
        && getBlockedRoads().contains( location().getID() ) ) {
      return true;
    }
    return false;
  }
  
  
  /**
   * Write information about the rescued victims into a file
   */
  @Override
  public void shutdown() {
    
    try {
      BufferedWriter file = new BufferedWriter(
          new FileWriter( this.outputDir + File.separator + "ambulance"
              + this.getID().getValue() + ".csv", true ) );
      
      String header = "id;sTime;sDist;sHP;sDamage;eTime;eDist;eHP;eDamage\n";
      file.write( header );
      
      for ( Integer id : this.victimInfo.keySet() ) {
        Map<String, Integer> info = this.victimInfo.get( id );
        
        if ( info.containsKey( "eTime" ) ) {
          String content = id + ";" + info.get( "sTime" ) + ";"
              + info.get( "sDist" ) + ";" + info.get( "sHP" ) + ";"
              + info.get( "sDamage" ) + ";" + info.get( "eTime" ) + ";"
              + info.get( "eDist" ) + ";" + info.get( "eHP" ) + ";"
              + info.get( "eDamage" ) + "\n";
          file.write( content );
        }
      }
      
      file.close();
    } catch ( IOException e ) {
      e.printStackTrace();
    }
    
    super.shutdown();
  }
}