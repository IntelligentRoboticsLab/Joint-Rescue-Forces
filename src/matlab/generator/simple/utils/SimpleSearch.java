/*******************************************************
 * Simple Breadth-First Search
 * 
 * @copyright Copyright 2012 Laboratorio de Tecnicas Inteligentes
 * @author Annibal B. M. da Silva
 * @author Luis G. Nardin
 * @author Jaime S. Sichman
 * @license BSD 3-Clause "New" or "Revised" License
 ********************************************************/
package matlab.generator.simple.utils;

import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import matlab.generator.area.Sector;
import rescuecore2.misc.collections.LazyMap;
import rescuecore2.standard.entities.Area;
import rescuecore2.standard.entities.StandardWorldModel;
import rescuecore2.worldmodel.Entity;
import rescuecore2.worldmodel.EntityID;

public class SimpleSearch {
  
  private Map<EntityID, Set<EntityID>> graph;
  
  
  /**
   * Construct a new SampleSearch.
   * 
   * @param world
   *          The world model to construct the neighborhood graph from.
   */
  public SimpleSearch( StandardWorldModel world ) {
    Map<EntityID, Set<EntityID>> neighbours = new LazyMap<EntityID, Set<EntityID>>() {
      
      @Override
      public Set<EntityID> createValue() {
        return new HashSet<EntityID>();
      }
    };
    for ( Entity next : world ) {
      if ( next instanceof Area ) {
        Collection<EntityID> areaNeighbours = ((Area) next).getNeighbours();
        neighbours.get( next.getID() ).addAll( areaNeighbours );
      }
    }
    setGraph( neighbours );
  }
  
  
  /**
   * Construct a new ConnectionGraphSearch.
   * 
   * @param graph
   *          The connection graph in the form of a map from EntityID to the set
   *          of neighbouring EntityIDs.
   */
  public SimpleSearch( Map<EntityID, Set<EntityID>> graph ) {
    setGraph( graph );
  }
  
  
  /**
   * Set the neighbourhood graph.
   * 
   * @param newGraph
   *          The new neighbourhood graph.
   */
  public void setGraph( Map<EntityID, Set<EntityID>> newGraph ) {
    this.graph = newGraph;
  }
  
  
  /**
   * Get the neighbourhood graph.
   * 
   * @return The neighbourhood graph.
   */
  public Map<EntityID, Set<EntityID>> getGraph() {
    return graph;
  }
  
  
  /**
   * Do a breadth first search from one location to the closest (in terms of
   * number of nodes) of a set of goals.
   * 
   * @param start
   *          The location we start at.
   * @param goals
   *          The set of possible goals.
   * @return The path from start to one of the goals, or null if no path can be
   *         found.
   */
  public List<EntityID> breadthFirstSearch( EntityID start,
      EntityID... goals ) {
    return breadthFirstSearch( start, Arrays.asList( goals ) );
  }
  
  
  /**
   * Do a breadth first search from one location to the closest (in terms of
   * number of nodes) of a set of goals.
   * 
   * @param start
   *          The location we start at.
   * @param goals
   *          The set of possible goals.
   * @return The path from start to one of the goals, or null if no path can be
   *         found.
   */
  public List<EntityID> breadthFirstSearch( EntityID start,
      Collection<EntityID> goals ) {
    List<EntityID> open = new LinkedList<EntityID>();
    Map<EntityID, EntityID> ancestors = new HashMap<EntityID, EntityID>();
    open.add( start );
    EntityID next = null;
    boolean found = false;
    ancestors.put( start, start );
    do {
      next = open.remove( 0 );
      if ( isGoal( next, goals ) ) {
        found = true;
        break;
      }
      Collection<EntityID> neighbours = graph.get( next );
      if ( neighbours.isEmpty() ) {
        continue;
      }
      for ( EntityID neighbour : neighbours ) {
        if ( isGoal( neighbour, goals ) ) {
          ancestors.put( neighbour, next );
          next = neighbour;
          found = true;
          break;
        } else {
          if ( !ancestors.containsKey( neighbour ) ) {
            open.add( neighbour );
            ancestors.put( neighbour, next );
          }
        }
      }
    } while ( !found && !open.isEmpty() );
    if ( !found ) {
      // No path
      return null;
    }
    // Walk back from goal to start
    EntityID current = next;
    List<EntityID> path = new LinkedList<EntityID>();
    do {
      path.add( 0, current );
      current = ancestors.get( current );
      if ( current == null ) {
        throw new RuntimeException(
            "Found a node with no ancestor! Something is broken." );
      }
    } while ( current != start );
    return path;
  }
  
  
  private boolean isGoal( EntityID e, Collection<EntityID> test ) {
    return test.contains( e );
  }
  
  
  public List<EntityID> breadthFirstSearch( EntityID start, Sector sector,
      EntityID... goals ) {
    return breadthFirstSearch( start, sector, Arrays.asList( goals ) );
  }
  
  
  public List<EntityID> breadthFirstSearch( EntityID start, Sector sector,
      Collection<EntityID> goals ) {
    List<EntityID> open = new LinkedList<EntityID>();
    Map<EntityID, EntityID> ancestors = new HashMap<EntityID, EntityID>();
    open.add( start );
    EntityID next = null;
    boolean found = false;
    ancestors.put( start, start );
    do {
      next = open.remove( 0 );
      if ( isGoal( next, goals ) ) {
        found = true;
        break;
      }
      Collection<EntityID> neighbours = graph.get( next );
      if ( neighbours.isEmpty() ) {
        continue;
      }
      Set<EntityID> outside = new HashSet<EntityID>();
      for ( EntityID neighbour : neighbours ) {
        if ( !sector.getLocations().contains( neighbour ) ) {
          outside.add( neighbour );
        }
      }
      neighbours.removeAll( outside );
      for ( EntityID neighbour : neighbours ) {
        if ( isGoal( neighbour, goals ) ) {
          ancestors.put( neighbour, next );
          next = neighbour;
          found = true;
          break;
        } else {
          if ( !ancestors.containsKey( neighbour ) ) {
            open.add( neighbour );
            ancestors.put( neighbour, next );
          }
        }
      }
    } while ( !found && !open.isEmpty() );
    if ( !found ) {
      // No path
      return null;
    }
    // Walk back from goal to start
    EntityID current = next;
    List<EntityID> path = new LinkedList<EntityID>();
    do {
      path.add( 0, current );
      current = ancestors.get( current );
      if ( current == null ) {
        throw new RuntimeException(
            "Found a node with no ancestor! Something is broken." );
      }
    } while ( current != start );
    return path;
  }
  
  
  public List<EntityID> pathFinder( EntityID start,
      Collection<EntityID> blockedRoads, EntityID... goals ) {
    return pathFinder( start, blockedRoads, Arrays.asList( goals ) );
  }
  
  
  public List<EntityID> pathFinder( EntityID start,
      Collection<EntityID> blockedRoads, Collection<EntityID> goals ) {
    List<EntityID> open = new LinkedList<EntityID>();
    Map<EntityID, EntityID> ancestors = new HashMap<EntityID, EntityID>();
    open.add( start );
    EntityID next = null;
    boolean found = false;
    ancestors.put( start, start );
    
    do {
      next = open.remove( 0 );
      if ( isGoal( next, goals ) ) {
        found = true;
        break;
      }
      Collection<EntityID> neighbours = graph.get( next );
      if ( neighbours.isEmpty() ) {
        continue;
      }
      for ( EntityID neighbour : neighbours ) {
        if ( isGoal( neighbour, goals ) ) {
          ancestors.put( neighbour, next );
          next = neighbour;
          found = true;
          break;
        } else {
          if ( !ancestors.containsKey( neighbour )
              && !blockedRoads.contains( neighbour ) ) {
            open.add( neighbour );
            ancestors.put( neighbour, next );
          }
        }
      }
    } while ( !found && !open.isEmpty() );
    if ( !found ) {
      // No path
      return null;
    }
    // Walk back from goal to start
    EntityID current = next;
    List<EntityID> path = new LinkedList<EntityID>();
    do {
      path.add( 0, current );
      current = ancestors.get( current );
      if ( current == null ) {
        throw new RuntimeException(
            "Found a node with no ancestor! Something is broken." );
      }
    } while ( current != start );
    
    if ( path.size() <= 2 ) {
      return null;
    }
    return path;
  }
}