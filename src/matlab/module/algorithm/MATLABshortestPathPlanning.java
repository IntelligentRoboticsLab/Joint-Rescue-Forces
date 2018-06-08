package matlab.module.algorithm;

import adf.agent.develop.DevelopData;
import adf.agent.info.AgentInfo;
import adf.agent.info.ScenarioInfo;
import adf.agent.info.WorldInfo;
import adf.agent.module.ModuleManager;
import adf.agent.precompute.PrecomputeData;
import adf.component.module.algorithm.PathPlanning;
import rescuecore2.misc.collections.LazyMap;
import rescuecore2.standard.entities.*;
import rescuecore2.misc.Pair;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.worldmodel.Entity;
import rescuecore2.worldmodel.EntityID;

import java.util.*;

import adf.launcher.ConsoleOutput;

import com.mathworks.engine.MatlabEngine;

import java.text.DecimalFormat;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;

       // This is the Joint Rescue Force code for the Machine Learning workshop - developed April 2018
       // Luis Gustavo Nardin, Sebastian Castro and Arnoud Visser

public class MATLABshortestPathPlanning extends PathPlanning {

   private EntityID from;
   private Collection<EntityID> targets;
   private List<EntityID> result;
   private MatlabEngine ml;

   public MATLABshortestPathPlanning(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager, DevelopData developData) {
       super(ai, wi, si, moduleManager, developData);
       
       System.out.println("[" + this.getClass().getSimpleName() + "] starting up");
       try {
            String matlab_command;
         //Start MATLAB asynchronously
            Future<MatlabEngine> eng = MatlabEngine.startMatlabAsync();

         // Get engine instance from the future result
            this.ml = eng.get();

         // Make sure that you are in the right directory
            this.ml.eval("cd('precomp_data');");

	    Future<String> future_string = this.ml.fevalAsync("pwd");
	    String pwd = future_string.get();
	    System.out.println("[" + this.getClass().getSimpleName() + "] MatlabEngine started in: " + pwd);
	} catch (ExecutionException | InterruptedException e) {
	    e.printStackTrace();
	}

   }

   @Override
   public PathPlanning precompute(PrecomputeData precomputeData) {
       super.precompute(precomputeData);
       System.out.println("[" + this.getClass().getSimpleName() + "] calling precompute");
       if(this.getCountPrecompute() >= 2) {
           return this;
       }

       this.matlab_init();

       // example to write other precompute data:
       //precomputeData.setBoolean(KEY_ASSIGN_AGENT, this.assignAgentsFlag);
       return this;
   }

   private void matlab_init() {

       try {
            String matlab_command;

         // Create empty table
            this.ml.eval("nodeTable = table([],[],[],'VariableNames',{'ID', 'X','Y'});G=graph([],[],[],nodeTable);"); 

         // First create all nodes, so that you can connect them by name.
            for (Entity next : this.worldInfo.getEntitiesOfType(
                StandardEntityURN.ROAD,
                StandardEntityURN.HYDRANT,
                StandardEntityURN.BUILDING,
                StandardEntityURN.REFUGE,
                StandardEntityURN.GAS_STATION,
                StandardEntityURN.AMBULANCE_CENTRE,
                StandardEntityURN.FIRE_STATION,
                StandardEntityURN.POLICE_OFFICE)) {

                Pair loc = this.worldInfo.getLocation(next.getID());
                matlab_command = String.format("G=addnode(G,table(\"%d\",%d,%d,'VariableNames',{'ID' 'X','Y'}));",next.getID().getValue(),loc.first(),loc.second());
                this.ml.eval(matlab_command);
            }

            for (Entity next : this.worldInfo.getEntitiesOfType(
                StandardEntityURN.ROAD,
                StandardEntityURN.HYDRANT,
                StandardEntityURN.BUILDING,
                StandardEntityURN.REFUGE,
                StandardEntityURN.GAS_STATION,
                StandardEntityURN.AMBULANCE_CENTRE,
                StandardEntityURN.FIRE_STATION,
                StandardEntityURN.POLICE_OFFICE)) {
                if (next instanceof Area) {
                    Collection<EntityID> areaNeighbours = ((Area) next).getNeighbours();

                   for(EntityID neighbour : areaNeighbours) {
                         if (next.getID().getValue()  > neighbour.getValue()) { // only do it once
                            matlab_command = String.format("G=addedge(G,find(G.Nodes.ID==\"%d\"), find(G.Nodes.ID==\"%d\"), %d);",
                                   next.getID().getValue(),neighbour.getValue(),this.worldInfo.getDistance(next.getID(), neighbour)); // by name, not by index

                            this.ml.eval(matlab_command);
			  }
		   }
		 }
            }
		 
	    this.ml.eval("save('graph.mat', 'G', '-append');");

	    Future<String> future_string = this.ml.fevalAsync("pwd");
	    String pwd = future_string.get();
	    System.out.println("[" + this.getClass().getSimpleName() + "] saving graph.mat in: " + pwd);
		 
	 // Disconnect from the MATLAB session
	    ml.disconnect();

	} catch (ExecutionException | InterruptedException e) {
	    e.printStackTrace();
	}

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
   public PathPlanning resume(PrecomputeData precomputeData) {
       super.resume(precomputeData);
     
       System.out.println("[" + this.getClass().getSimpleName() + "] calling resume");
       return this;
   }

   @Override
   public PathPlanning preparate() {
       super.preparate();
//       System.out.println("[" + this.getClass().getSimpleName() + "] calling preparate");
       return this;
   }

   @Override
   public PathPlanning calc() {
       List<EntityID> path = new LinkedList<>();

       System.out.println("[" + this.getClass().getSimpleName() + "] planning from location with ID: " + this.from + " to " + this.targets);

       try {
	    String matlab_command;

	 // Start MATLAB asynchronously
	 // Future<MatlabEngine> eng = MatlabEngine.startMatlabAsync(); // already done in constructor

	 // Get engine instance from the future result
	 //   MatlabEngine ml = eng.get();


	    String from_string = String.format("%d",this.from.getValue());
	    System.out.println("[" + this.getClass().getSimpleName() + "] math_calc(): from:"  + from_string );

	    int target_length = this.targets.size();
            if(target_length < 2) {
	       System.out.println("[" + this.getClass().getSimpleName() + "] math_calc(): only:"  + this.targets );
             }
             String[] targetlist = new String[target_length];
             int t=0;
	     for (Iterator<EntityID> iterator = this.targets.iterator(); iterator.hasNext();) {
			EntityID target = iterator.next(); 
			targetlist[t++] = String.valueOf(target.getValue());
	    }

             String[] mlOutput = this.ml.feval("getPath",from_string,targetlist); // calls getpath.m

             for (int i= mlOutput.length - 1; i >= 0; i--) {  

                String waypointS = mlOutput[i];
	//	System.out.println("[" + this.getClass().getSimpleName() + "] math_calc(): executing: get_path() gave result: " + waypointS);

                EntityID waypoint_entity = new rescuecore2.worldmodel.EntityID(Integer.parseInt(waypointS));
	        path.add(0, waypoint_entity);
             }

		 // Disconnect from the MATLAB session
		 //   ml.disconnect();
	} catch (ExecutionException | InterruptedException e) {
		    e.printStackTrace();
	}
	//  return found path
	this.result = path;

	System.out.println("[" + this.getClass().getSimpleName() + "] found a path:");
	for (int i = 0; i < path.size(); i++) {
	       System.out.println(i + ": " + path.get(i));
	}
	return this;
   }
}
