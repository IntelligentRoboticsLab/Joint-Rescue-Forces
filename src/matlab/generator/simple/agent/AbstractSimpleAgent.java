/*******************************************************
 * Abstract Simple Agent
 * 
 * @copyright Copyright 2012 Laboratorio de Tecnicas Inteligentes
 * @author Annibal B. M. da Silva
 * @author Luis G. Nardin
 * @author Jaime S. Sichman
 * @license BSD 3-Clause "New" or "Revised" License
 ********************************************************/
package matlab.generator.simple.agent;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import rescuecore2.misc.Pair;
import rescuecore2.misc.geometry.GeometryTools2D;
import rescuecore2.misc.geometry.Line2D;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.standard.components.StandardAgent;
import rescuecore2.standard.entities.Blockade;
import rescuecore2.standard.entities.Building;
import rescuecore2.standard.entities.Human;
import rescuecore2.standard.entities.Road;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.standard.entities.StandardEntityURN;
import rescuecore2.worldmodel.ChangeSet;
import rescuecore2.worldmodel.EntityID;
import matlab.generator.simple.utils.SimpleSearch;

public abstract class AbstractSimpleAgent<E extends StandardEntity>
    extends StandardAgent<E> {
  
  protected static final int             RANDOM_WALK_LENGTH = 50;
  
  private static final String            MAX_SIGHT_KEY      = "perception.los.max-distance";
  
  protected boolean                      logData;
  
  protected SimpleSearch                 search;
  
  protected Set<EntityID>                buildingIDs;
  
  protected Set<EntityID>                roadIDs;
  
  protected Map<EntityID, Set<EntityID>> neighbours;
  
  protected int                          maxSight;
  
  protected EntityID                     lastPosition;
  
  protected EntityID                     currentPosition;
  
  protected EntityID                     target;
  
  protected String                       outputDir;
  
  
  @Override
  protected void postConnect() {
    super.postConnect();
    
    model.indexClass( StandardEntityURN.BUILDING );
    buildingIDs = new HashSet<EntityID>();
    roadIDs = new HashSet<EntityID>();
    
    try {
      BufferedWriter entitiesWriter = new BufferedWriter( new FileWriter(
          this.outputDir + File.separator + "entities.csv", true ) );
      
      entitiesWriter.write( "entityID;x;y\n" );
      BufferedWriter buildingsWriter = new BufferedWriter( new FileWriter(
          this.outputDir + File.separator + "buildings.csv", true ) );
      buildingsWriter.write( "buildingID;x;y;totalArea;floors\n" );
      
      for ( StandardEntity next : model ) {
        String contentE = "";
        String contentB = "";
        if ( next instanceof Building ) {
          buildingIDs.add( next.getID() );
          Pair<Integer, Integer> location = next.getLocation( model );
          contentE = next.getID().toString() + ";" + location.first() + ";"
              + location.second() + "\n";
          contentB = next.getID().toString() + ";" + location.first() + ";"
              + location.second() + ";" + ((Building) next).getTotalArea() + ";"
              + ((Building) next).getFloors() + "\n";
        } else if ( next instanceof Road ) {
          roadIDs.add( next.getID() );
          Pair<Integer, Integer> location = next.getLocation( model );
          contentE = next.getID().toString() + ";" + location.first() + ";"
              + location.second() + "\n";
        }
        
        if ( !contentE.isEmpty() ) {
          entitiesWriter.write( contentE );
        }
        if ( !contentB.isEmpty() ) {
          buildingsWriter.write( contentB );
        }
      }
      
      entitiesWriter.close();
      buildingsWriter.close();
      
    } catch ( IOException e ) {
    }
    
    search = new SimpleSearch( model );
    
    neighbours = search.getGraph();
    
    maxSight = config.getIntValue( MAX_SIGHT_KEY );
    
    lastPosition = location().getID();
    
    currentPosition = lastPosition;
    
    target = null;
  }
  
  
  protected List<EntityID> randomWalk() {
    List<EntityID> result = new ArrayList<EntityID>( RANDOM_WALK_LENGTH );
    Set<EntityID> seen = new HashSet<EntityID>();
    EntityID current = ((Human) me()).getPosition();
    for ( int i = 0; i < RANDOM_WALK_LENGTH; ++i ) {
      result.add( current );
      seen.add( current );
      List<EntityID> possible = new ArrayList<EntityID>();
      boolean found = false;
      
      for ( EntityID next : neighbours.get( current ) ) {
        if ( model.getEntity( next ) instanceof Building ) {
          if ( !((Building) model.getEntity( next )).isOnFire() ) {
            possible.add( next );
          }
        } else {
          possible.add( next );
        }
      }
      
      Collections.shuffle( possible, random );
      
      for ( EntityID next : possible ) {
        if ( seen.contains( next ) ) {
          continue;
        }
        current = next;
        found = true;
        break;
      }
      if ( !found ) {
        // We reached a dead-end.
        break;
      }
    }
    return result;
  }
  
  
  protected Set<EntityID> getSafeBuildings( ChangeSet changed ) {
    Set<EntityID> buildings = getVisibleEntitiesOfType(
        StandardEntityURN.BUILDING, changed );
    Set<EntityID> result = new HashSet<EntityID>();
    
    for ( EntityID next : buildings ) {
      StandardEntity e = model.getEntity( next );
      if ( !((Building) e).isOnFire() ) {
        result.add( next );
      }
    }
    return result;
  }
  
  
  /**
   * Identify visible entities of a specific type in the perception
   * 
   * @param type
   *          Entity type
   * @param changed
   *          Perception of changes in the environment
   * 
   * @return List of visible entities of the specified type
   */
  protected Set<EntityID> getVisibleEntitiesOfType( StandardEntityURN type,
      ChangeSet changed ) {
    Set<EntityID> visibleEntities = changed.getChangedEntities();
    Set<EntityID> result = new HashSet<EntityID>();
    
    for ( EntityID next : visibleEntities ) {
      if ( model.getEntity( next ).getStandardURN().equals( type ) ) {
        result.add( next );
      }
    }
    return result;
  }
  
  
  /**
   * Calculate the distance to a blockade
   * 
   * @param b
   *          Blockade
   * @param x
   *          Initial x-axis location
   * @param y
   *          Initial y-axis location
   * 
   * @return Distance to a blockade
   */
  protected int findDistanceTo( Blockade b, int x, int y ) {
    // Logger.debug("Finding distance to " + b + " from " + x + ", " + y);
    List<Line2D> lines = GeometryTools2D.pointsToLines(
        GeometryTools2D.vertexArrayToPoints( b.getApexes() ), true );
    double best = Double.MAX_VALUE;
    Point2D origin = new Point2D( x, y );
    for ( Line2D next : lines ) {
      Point2D closest = GeometryTools2D.getClosestPointOnSegment( next,
          origin );
      double d = GeometryTools2D.getDistance( origin, closest );
      // Logger.debug("Next line: " + next + ", closest point: " + closest
      // + ", distance: " + d);
      if ( d < best ) {
        best = d;
        // Logger.debug("New best distance");
      }
    }
    return (int) best;
  }
  
  
  /**
   * Identify know blockade roads
   * 
   * @return List of known blocked roads
   */
  protected List<EntityID> getBlockedRoads() {
    Collection<StandardEntity> e = model
        .getEntitiesOfType( StandardEntityURN.ROAD );
    List<EntityID> result = new ArrayList<EntityID>();
    for ( StandardEntity next : e ) {
      Road r = (Road) next;
      if ( r.isBlockadesDefined() && !r.getBlockades().isEmpty() ) {
        result.add( r.getID() );
      }
    }
    return result;
  }
  
  
  /**
   * Indicates that the Building and Road entries should be written in a file
   */
  public void logData() {
    this.logData = true;
  }
}