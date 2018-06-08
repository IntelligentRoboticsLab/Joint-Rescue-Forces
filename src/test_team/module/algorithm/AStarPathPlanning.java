package test_team.module.algorithm;

import adf.agent.develop.DevelopData;
import adf.agent.info.AgentInfo;
import adf.agent.info.ScenarioInfo;
import adf.agent.info.WorldInfo;
import adf.agent.module.ModuleManager;
import adf.agent.precompute.PrecomputeData;
import adf.component.module.algorithm.PathPlanning;
import rescuecore2.misc.collections.LazyMap;
import rescuecore2.standard.entities.Area;
import rescuecore2.worldmodel.Entity;
import rescuecore2.worldmodel.EntityID;

import java.util.*;

public class AStarPathPlanning extends PathPlanning {

   private Map<EntityID, Set<EntityID>> graph;

   private EntityID from;
   private Collection<EntityID> targets;
   private List<EntityID> result;

   public AStarPathPlanning(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager, DevelopData developData) {
       super(ai, wi, si, moduleManager, developData);
       this.init();
   }

   private void init() {
       Map<EntityID, Set<EntityID>> neighbours = new LazyMap<EntityID, Set<EntityID>>() {
           @Override
           public Set<EntityID> createValue() {
               return new HashSet<>();
           }
       };
       for (Entity next : this.worldInfo) {
           if (next instanceof Area) {
               Collection<EntityID> areaNeighbours = ((Area) next).getNeighbours();
               neighbours.get(next.getID()).addAll(areaNeighbours);
           }
       }
       this.graph = neighbours;
   }

   @Override
   public List<EntityID> getResult() {
       return this.result;
   }

   @Override
   public PathPlanning setFrom(EntityID id) {
       this.from = id;
       return this;
   }

   @Override
   public PathPlanning setDestination(Collection<EntityID> targets) {
     this.targets = targets;
     return this;
   }

   @Override
   public PathPlanning precompute(PrecomputeData precomputeData) {
       super.precompute(precomputeData);
       return this;
   }

   @Override
   public PathPlanning resume(PrecomputeData precomputeData) {
       super.resume(precomputeData);
       return this;
   }

   @Override
   public PathPlanning preparate() {
       super.preparate();
       return this;
   }

   @Override
   public PathPlanning calc() {
       // see Fig 3.13 of Russell & Norvig
	   
       System.out.println("[" + this.getClass().getSimpleName() + "] planning from location with ID: " + this.from + " to " + this.targets);

       //  1
       List<EntityID> open = new LinkedList<>(); // the frontier
       List<EntityID> close = new LinkedList<>(); // already explored nodes 
       Map<EntityID, Node> nodeMap = new HashMap<>(); 
    
       //  3
       open.add(this.from); // the frontier is initiated with the start-node
       nodeMap.put(this.from, new Node(null, this.from));
       close.clear(); // the explored list starts empty
    
       while (true) {
           //  4
           if (open.size() < 0) {
               this.result = null; // failure - no path possible
               return this;
           }
    
           //  5
           Node n = null;
           for (EntityID id : open) { // search nearest neighbor, because the frontier is not a priority queue ordered by Path-Cost
               Node node = nodeMap.get(id);
    
               if (n == null) {
                   n = node;
               } else if (node.estimate() < n.estimate()) { // choose the lowest-cost node in the frontier
                   n = node;
               }
           }
    
           //  6
           if (targets.contains(n.getID())) {
               //  9
               List<EntityID> path = new LinkedList<>();
               while (n != null) {
                   path.add(0, n.getID());
                   n = nodeMap.get(n.getParent());
               }
    
               this.result = path;

               System.out.println("[" + this.getClass().getSimpleName() + "] found a path:");
               for (int i = 0; i < path.size(); i++) {
                   System.out.println(i + ": " + path.get(i));
               }
               return this; // return solution
           }
           open.remove(n.getID()); // no queue, so node has to be explictly removed from frontier
           close.add(n.getID());   // and added to explored list (to prevent cycles)
    
           //  7
           Collection<EntityID> neighbours = this.graph.get(n.getID());
           for (EntityID neighbour : neighbours) { // check each child
               Node m = new Node(n, neighbour);
    
               if (!open.contains(neighbour) && !close.contains(neighbour)) { // nor in frontier nor in explored list
                   open.add(m.getID());
                   nodeMap.put(neighbour, m);
               } // if child is in frontier with higher Path-Cost replace child (found a short-cut)
               else if (open.contains(neighbour) && m.estimate() < nodeMap.get(neighbour).estimate()) {
                   nodeMap.put(neighbour, m);
               }
               else if (!close.contains(neighbour) && m.estimate() < nodeMap.get(neighbour).estimate()) {
                   nodeMap.put(neighbour, m);
               }
           }
       }
   }


   private class Node {
	    EntityID id;
	    EntityID parent;
	 
	    double cost;
	    double heuristic;
	 
	    public Node(Node from, EntityID id) {
	        this.id = id;
	 
	        if (from == null) {
	            this.cost = 0;
	        } else {
	            this.parent = from.getID();
	            this.cost = from.getCost() + worldInfo.getDistance(from.getID(), id);
	        }
	 
	        this.heuristic = worldInfo.getDistance(id, targets.toArray(new EntityID[targets.size()])[0]);
	    }
	 
	    public EntityID getID() {
	        return id;
	    }
	 
	    public double getCost() {
	        return cost;
	    }
	 
	    public double estimate() {
	        return cost + heuristic;
	    }
	 
	    public EntityID getParent() {
	        return this.parent;
	    }
	}
}
