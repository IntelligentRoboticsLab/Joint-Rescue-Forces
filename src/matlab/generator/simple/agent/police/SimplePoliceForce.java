/*******************************************************
 * Simple Police Force
 * 
 * @copyright Copyright 2012 Laboratorio de Tecnicas Inteligentes
 * @author Annibal B. M. da Silva
 * @author Luis G. Nardin
 * @author Jaime S. Sichman
 * @license BSD 3-Clause "New" or "Revised" License
 ********************************************************/
package matlab.generator.simple.agent.police;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;
import java.util.TreeSet;
import matlab.generator.area.Sector;
import matlab.generator.simple.agent.AbstractSimpleAgent;
import rescuecore2.messages.Command;
import rescuecore2.misc.Pair;
import rescuecore2.standard.entities.Area;
import rescuecore2.standard.entities.Blockade;
import rescuecore2.standard.entities.Building;
import rescuecore2.standard.entities.Human;
import rescuecore2.standard.entities.PoliceForce;
import rescuecore2.standard.entities.Road;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.standard.entities.StandardEntityURN;
import rescuecore2.worldmodel.ChangeSet;
import rescuecore2.worldmodel.EntityID;

public class SimplePoliceForce extends AbstractSimpleAgent<PoliceForce> {
  
  private static final String DISTANCE_KEY = "clear.repair.distance";
  
  private static enum State {
    MOVING_TO_ROAD,
    RANDOM_WALKING,
    MOVING_TO_BLOCKADE,
    CLEARING,
    RETURNING_TO_SECTOR,
    BURIED,
    DEAD
  };
  
  private int    distance;
  
  private Sector sector;
  
  @SuppressWarnings ( "unused" )
  private State  state;
  
  
  /**
   * Constructor
   * 
   * @param outputDir
   *          Output directory
   */
  public SimplePoliceForce( String outputDir ) {
    this.outputDir = outputDir;
  }
  
  
  /**
   * Define the agent as a Police Force
   */
  @Override
  protected EnumSet<StandardEntityURN> getRequestedEntityURNsEnum() {
    return EnumSet.of( StandardEntityURN.POLICE_FORCE );
  }
  
  
  /**
   * Executed after connected to the simulator (1) initializes global variables
   * (2) extracts information about sectors to search
   */
  @Override
  protected void postConnect() {
    super.postConnect();
    
    distance = config.getIntValue( DISTANCE_KEY );
    
    Set<Sector> sectors = sectorize();
    
    Set<EntityID> policeForces = new TreeSet<EntityID>(
        new EntityIDComparator() );
    
    for ( StandardEntity e : model
        .getEntitiesOfType( StandardEntityURN.POLICE_FORCE ) ) {
      policeForces.add( e.getID() );
    }
    
    List<EntityID> policeForcesList = new ArrayList<EntityID>( policeForces );
    List<Sector> sectorsList = new ArrayList<Sector>( sectors );
    
    sector = sectorsList.get(
        policeForcesList.indexOf( me().getID() ) % (policeForces.size() / 2) );
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
    if ( me().getHP() == 0 ) {
      state = State.DEAD;
      return;
    }
    
    if ( me().getBuriedness() != 0 ) {
      state = State.BURIED;
      return;
    }
    
    if ( location() instanceof Building ) {
      if ( ((Building) location()).isOnFire() ) {
        List<EntityID> path = randomWalk();
        
        sendMove( time, path );
        return;
      }
    }
    
    Set<EntityID> blockades = getVisibleEntitiesOfType(
        StandardEntityURN.BLOCKADE, changed );
    EntityID target = getTargetBlockade( blockades );
    
    // Is there any blockade in range?
    if ( target != null ) {
      sendClear( time, target );
      state = State.CLEARING;
      return;
    }
    
    List<EntityID> path = null;
    
    // Have I seen any blockade at all?
    if ( !blockades.isEmpty() ) {
      path = search.breadthFirstSearch( me().getPosition(), blockades );
      if ( path != null ) {
        sendMove( time, path );
        state = State.MOVING_TO_BLOCKADE;
        return;
      }
    } else {
      // Look for blockades on the visible roads
      for ( EntityID next : getVisibleEntitiesOfType( StandardEntityURN.ROAD,
          changed ) ) {
        if ( ((Road) model.getEntity( next )).isBlockadesDefined()
            && !((Road) model.getEntity( next )).getBlockades().isEmpty() ) {
          path = search.breadthFirstSearch( me().getPosition(), next );
          if ( path != null ) {
            sendMove( time, path );
            state = State.MOVING_TO_BLOCKADE;
            return;
          }
        }
      }
      
      /*
       * Check a road of this sector in which a blockade was identified a while
       * ago
       */
      Set<EntityID> blockedRoads = new HashSet<EntityID>();
      
      for ( EntityID next : getBlockedRoads() ) {
        if ( sector.getLocations().contains( next ) ) {
          blockedRoads.add( next );
        }
      }
      
      if ( !blockedRoads.isEmpty() ) {
        path = search.breadthFirstSearch( me().getPosition(), sector,
            blockedRoads );
      }
      
      if ( path != null ) {
        state = State.MOVING_TO_ROAD;
        sendMove( time, path );
        return;
      }
    }
    
    // Patrol the sector
    if ( sector.getLocations().contains( me().getPosition() ) ) {
      path = randomWalk( time );
      state = State.RANDOM_WALKING;
    } else {
      path = search.breadthFirstSearch( me().getPosition(),
          (EntityID) sector.getLocations().toArray()[0] );
      state = State.RETURNING_TO_SECTOR;
    }
    
    sendMove( time, path );
    return;
  }
  
  
  private EntityID getTargetBlockade( Set<EntityID> blockades ) {
    int x = me().getX();
    int y = me().getY();
    
    for ( EntityID next : blockades ) {
      if ( findDistanceTo( (Blockade) model.getEntity( next ), x,
          y ) < distance ) {
        return next;
      }
    }
    
    return null;
  }
  
  
  /**
   * Divides the map into sectors, assigning the "Area" entities contained in
   * them to each Police Force.
   * 
   * @param neighbours
   *          The table of adjacency of the graph to be sectorized.
   **/
  private Set<Sector> sectorize() {
    Pair<Pair<Integer, Integer>, Pair<Integer, Integer>> bounds = model
        .getWorldBounds();
    
    // Get the two points that define the map as rectangle.
    int minX = bounds.first().first();
    int minY = bounds.first().second();
    int maxX = bounds.second().first();
    int maxY = bounds.second().second();
    int numberOfSectors = model
        .getEntitiesOfType( StandardEntityURN.POLICE_FORCE ).size() / 2;
    
    int length = maxX - minX;
    int height = maxY - minY;
    
    // Get the number of divisions on each dimension.
    Pair<Integer, Integer> factors = factorization( numberOfSectors );
    
    int lengthDivisions;
    int heightDivisions;
    
    if ( length < height ) {
      lengthDivisions = factors.second();
      heightDivisions = factors.first();
    } else {
      lengthDivisions = factors.first();
      heightDivisions = factors.second();
    }
    
    // Divide the map into sectors
    Set<Sector> sectors = new TreeSet<Sector>();
    
    for ( int i = 1; i <= heightDivisions; i++ ) {
      for ( int j = 1; j <= lengthDivisions; j++ ) {
        Sector s = new Sector( minX + (length * (j - 1)) / lengthDivisions,
            minY + (height * (i - 1)) / heightDivisions,
            minX + (length * j) / lengthDivisions,
            minY + (height * i) / heightDivisions,
            lengthDivisions * (i - 1) + j );
        sectors.add( s );
      }
    }
    
    sectors = allocateNodes( sectors );
    
    return sectors;
  }
  
  
  /**
   * Allocates a set of connected entities to each sector.
   * 
   * @param sectors
   *          The set of sectors the entities should be allocated between.
   * @return The set of sectors received, with the entities allocated.
   */
  private Set<Sector> allocateNodes( Set<Sector> sectors ) {
    Map<Integer, Set<Set<EntityID>>> connectedGraphs = new HashMap<Integer, Set<Set<EntityID>>>();
    
    for ( Sector s : sectors ) {
      Collection<StandardEntity> entities = model.getObjectsInRectangle(
          s.getBounds().first().first(), s.getBounds().first().second(),
          s.getBounds().second().first(), s.getBounds().second().second() );
      
      Set<EntityID> locations = new HashSet<EntityID>();
      
      // Determine the entities geographically contained in the sector
      for ( StandardEntity next : entities ) {
        if ( next instanceof Area ) {
          if ( s.geographicallyContains( (Area) next ) ) {
            locations.add( next.getID() );
          }
        }
      }
      
      Set<Set<EntityID>> subsets = new HashSet<Set<EntityID>>();
      Set<EntityID> visited = new HashSet<EntityID>();
      List<EntityID> list = new ArrayList<EntityID>( locations );
      
      /*
       * Divide the entities contained in subgraphs
       */
      while ( !visited.equals( locations ) ) {
        Set<EntityID> connected = new HashSet<EntityID>();
        Set<EntityID> border = new HashSet<EntityID>();
        border.add( list.get( 0 ) );
        
        // Expand each subgraph
        while ( !border.isEmpty() ) {
          Set<EntityID> newBorder = new HashSet<EntityID>();
          
          for ( EntityID e : border ) {
            for ( EntityID next : neighbours.get( e ) ) {
              if ( locations.contains( next ) && !visited.contains( next )
                  && !border.contains( next ) ) {
                newBorder.add( next );
              }
            }
            visited.add( e );
          }
          connected.addAll( border );
          list.removeAll( border );
          border = newBorder;
        }
        subsets.add( connected );
      }
      connectedGraphs.put( s.getIndex(), subsets );
    }
    
    // Get the largest connected graph of each sector.
    for ( Sector s : sectors ) {
      int maxSize = 0;
      Set<EntityID> location = null;
      
      for ( Set<EntityID> next : connectedGraphs.get( s.getIndex() ) ) {
        if ( next.size() >= maxSize ) {
          maxSize = next.size();
          location = next;
        }
      }
      
      s.setLocations( location );
      connectedGraphs.get( s.getIndex() ).remove( location );
    }
    
    int numberOfSectors = model
        .getEntitiesOfType( StandardEntityURN.POLICE_FORCE ).size();
    
    // Allocate the remaining subgraphs
    while ( !connectedGraphs.isEmpty() ) {
      for ( int i = 1; i <= numberOfSectors; i++ ) {
        if ( connectedGraphs.containsKey( i ) ) {
          List<Set<EntityID>> allocated = new ArrayList<Set<EntityID>>();
          
          for ( Set<EntityID> g : connectedGraphs.get( i ) ) {
            int minSectorSize = Integer.MAX_VALUE;
            Sector sector = null;
            
            for ( EntityID v : g ) {
              for ( EntityID next : neighbours.get( v ) ) {
                for ( Sector s : sectors ) {
                  if ( s.getLocations().contains( next ) ) {
                    if ( minSectorSize > s.getLocations().size() ) {
                      minSectorSize = s.getLocations().size();
                      sector = s;
                    }
                  }
                }
              }
            }
            if ( sector != null ) {
              sector.addVertices( g );
              allocated.add( g );
            }
          }
          
          while ( !allocated.isEmpty() ) {
            Set<EntityID> g = allocated.get( 0 );
            
            connectedGraphs.get( i ).contains( g );
            if ( connectedGraphs.get( i ).contains( g ) ) {
              connectedGraphs.get( i ).remove( g );
              allocated.remove( 0 );
            }
          }
          
          if ( connectedGraphs.get( i ).isEmpty() ) {
            connectedGraphs.remove( i );
          }
        }
      }
    }
    
    return sectors;
  }
  
  
  /**
   * Factor a number
   * 
   * @param n
   *          Number to factor
   * @return Factorized number
   */
  private Pair<Integer, Integer> factorization( int n ) {
    Pair<Integer, Integer> result = null;
    int difference = -1;
    
    for ( int i = 1; i <= (int) Math.sqrt( n ); i++ ) {
      if ( n % i == 0 ) {
        if ( -(n - n / i) < difference || difference == -1 ) {
          result = new Pair<Integer, Integer>( new Integer( i ),
              new Integer( n / i ) );
          difference = -(n - n / i);
        }
      }
    }
    
    return result;
  }
  
  
  /**
   * Random walk the agent
   * 
   * @param time
   *          Current timestep
   * @return List of entities representing the random walk path
   */
  private List<EntityID> randomWalk( int time ) {
    List<EntityID> result = new ArrayList<EntityID>( RANDOM_WALK_LENGTH );
    Set<EntityID> seen = new HashSet<EntityID>();
    EntityID current = ((Human) me()).getPosition();
    
    for ( int i = 0; i < RANDOM_WALK_LENGTH; ++i ) {
      result.add( current );
      seen.add( current );
      List<EntityID> possible = new ArrayList<EntityID>();
      
      for ( EntityID next : neighbours.get( current ) ) {
        if ( sector.getLocations().contains( next ) ) {
          possible.add( next );
        }
      }
      
      Collections.shuffle( possible,
          new Random( me().getID().getValue() + time ) );
      boolean found = false;
      
      for ( EntityID next : possible ) {
        if ( !seen.contains( next ) ) {
          current = next;
          found = true;
          break;
        }
      }
      if ( !found ) {
        // We reached a dead-end.
        break;
      }
    }
    
    List<EntityID> path = new ArrayList<EntityID>();
    for ( EntityID next : result ) {
      if ( model.getEntity( next ).getStandardURN()
          .compareTo( StandardEntityURN.BUILDING ) == 0
          && ((Building) model.getEntity( next )).isOnFire() ) {
        break;
      }
      path.add( next );
    }
    
    return path;
  }
}



class EntityIDComparator implements Comparator<EntityID> {
  
  public EntityIDComparator() {
  }
  
  
  @Override
  public int compare( EntityID a, EntityID b ) {
    
    if ( a.getValue() < b.getValue() ) {
      return -1;
    }
    if ( a.getValue() > b.getValue() ) {
      return 1;
    }
    return 0;
  }
}