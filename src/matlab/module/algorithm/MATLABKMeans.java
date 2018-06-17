package matlab.module.algorithm;

import adf.agent.communication.MessageManager;
import adf.agent.develop.DevelopData;
import adf.agent.info.AgentInfo;
import adf.agent.info.ScenarioInfo;
import adf.agent.info.WorldInfo;
import adf.agent.module.ModuleManager;
import adf.agent.precompute.PrecomputeData;
import adf.component.module.algorithm.Clustering;
import adf.component.module.algorithm.StaticClustering;
import rescuecore2.misc.Pair;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;
import com.mathworks.engine.MatlabEngine;
import java.util.concurrent.ExecutionException;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;

public class MATLABKMeans extends StaticClustering {
  
  /* CONSTANTS */
  private static final String                KEY_CLUSTER_SIZE   = "clustering.size";
  
  private static final String                KEY_CLUSTER_ENTITY = "clustering.entities.";
  
  private static final String                KEY_ASSIGN_AGENT   = "clustering.assign";
  
  private static final String                DISTANCE           = "distance";
  
  private static final String                MAX_ITER           = "maxIter";
  
  /* ATTRIBUTES */
  private Collection<StandardEntity>         entities;
  
  private Map<Integer, List<StandardEntity>> clusterEntitiesList;
  
  private List<List<EntityID>>               clusterEntityIDsList;
  
  private int                                clusterSize;
  
  private boolean                            assignAgentsFlag;
  
  private String                             distanceMetric;
  
  private double                             maxIter;
  
  
  /* METHODS */
  public MATLABKMeans( AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager, DevelopData developData ) {
    super( ai, wi, si, moduleManager, developData );
    
    this.clusterSize = developData
        .getInteger( "matlab.module.MATLABKMeans.clusterSize", 10 );
    this.assignAgentsFlag = developData
        .getBoolean( "matlab.module.MATLABKMeans.assignAgentsFlag", true );
    this.distanceMetric = developData
        .getString( "matlab.module.MATLABKMeans.distanceMetric", "cityblock" );
    this.maxIter = developData.getDouble( "matlab.module.MATLABKMeans.maxIter",
        100 );
    
    this.clusterEntityIDsList = new ArrayList<>();
    this.clusterEntitiesList = new HashMap<>();
    this.entities = wi.getEntitiesOfType( StandardEntityURN.ROAD,
        StandardEntityURN.HYDRANT, StandardEntityURN.BUILDING,
        StandardEntityURN.REFUGE, StandardEntityURN.GAS_STATION,
        StandardEntityURN.AMBULANCE_CENTRE, StandardEntityURN.FIRE_STATION,
        StandardEntityURN.POLICE_OFFICE );
  }
  
  
  @Override
  public Clustering updateInfo( MessageManager messageManager ) {
    super.updateInfo( messageManager );
    if ( this.getCountUpdateInfo() >= 2 ) {
      return this;
    }
    this.clusterEntitiesList.clear();
    return this;
  }
  
  
  @Override
  public Clustering precompute( PrecomputeData precomputeData ) {
    super.precompute( precomputeData );
    if ( this.getCountPrecompute() >= 2 ) {
      return this;
    }
    long startTime = System.currentTimeMillis();
    this.calcStandard();
    System.out.println( System.currentTimeMillis() - startTime );
    // write
    precomputeData.setInteger( KEY_CLUSTER_SIZE, this.clusterSize );
    for ( int i = 0; i < this.clusterSize; i++ ) {
      precomputeData.setEntityIDList( KEY_CLUSTER_ENTITY + i,
          this.clusterEntityIDsList.get( i ) );
    }
    precomputeData.setBoolean( KEY_ASSIGN_AGENT, this.assignAgentsFlag );
    return this;
  }
  
  
  @Override
  public Clustering resume( PrecomputeData precomputeData ) {
    super.resume( precomputeData );
    if ( this.getCountResume() >= 2 ) {
      return this;
    }
    // read
    this.clusterSize = precomputeData.getInteger( KEY_CLUSTER_SIZE );
    this.clusterEntityIDsList = new ArrayList<>( this.clusterSize );
    for ( int i = 0; i < this.clusterSize; i++ ) {
      this.clusterEntityIDsList.add( i,
          precomputeData.getEntityIDList( KEY_CLUSTER_ENTITY + i ) );
    }
    this.assignAgentsFlag = precomputeData.getBoolean( KEY_ASSIGN_AGENT );
    return this;
  }
  
  
  @Override
  public Clustering preparate() {
    super.preparate();
    if ( this.getCountPreparate() >= 2 ) {
      return this;
    }
    long startTime = System.currentTimeMillis();
    this.calcStandard();
    System.out.println( System.currentTimeMillis() - startTime );
    return this;
  }
  
  
  @Override
  public int getClusterNumber() {
    // The number of clusters
    return this.clusterSize;
  }
  
  
  @Override
  public int getClusterIndex( StandardEntity entity ) {
    return this.getClusterIndex( entity.getID() );
  }
  
  
  @Override
  public int getClusterIndex( EntityID id ) {
    for ( int i = 0; i < this.clusterSize; i++ ) {
      if ( this.clusterEntityIDsList.get( i ).contains( id ) ) {
        return i;
      }
    }
    return -1;
  }
  
  
  @Override
  public Collection<StandardEntity> getClusterEntities( int index ) {
    List<StandardEntity> result = this.clusterEntitiesList.get( index );
    if ( result == null || result.isEmpty() ) {
      List<EntityID> list = this.clusterEntityIDsList.get( index );
      result = new ArrayList<>( list.size() );
      for ( int i = 0; i < list.size(); i++ ) {
        result.add( i, this.worldInfo.getEntity( list.get( i ) ) );
      }
      this.clusterEntitiesList.put( index, result );
    }
    return result;
  }
  
  
  @Override
  public Collection<EntityID> getClusterEntityIDs( int index ) {
    return this.clusterEntityIDsList.get( index );
  }
  
  
  @Override
  public Clustering calc() {
    return this;
  }
  
  
  private void calcStandard() {
    int i = 0;
    
    try {
       MatlabEngine ml = MatlabEngine.startMatlab();
      
      // Prepare data for Matlab k-means++ clustering
      Map<String, EntityID> xyId = new HashMap<String, EntityID>();
      Pair<Integer, Integer> location;
      double[][] mlInput = new double[this.entities.size()][2];
      for ( StandardEntity entity : this.entities ) {
        location = this.worldInfo.getLocation( entity );
        
        mlInput[i][0] = location.first();
        mlInput[i][1] = location.second();
        
        xyId.put( Double.toString( mlInput[i][0] ) + ","
            + Double.toString( mlInput[i][1] ), entity.getID() );
        
        i++;
      }
      
      // Run k-means++ clustering
      Object[] mlOutput = ml.feval( 2, "kmeans", (Object) mlInput,
          this.clusterSize, DISTANCE, this.distanceMetric, MAX_ITER,
          this.maxIter );
      
      double[] mlIndex = (double[]) mlOutput[0];
      double[][] mlCenter = (double[][]) mlOutput[1];
      
      ml.close();
      
      this.clusterEntitiesList = new HashMap<Integer, List<StandardEntity>>(
          this.clusterSize );
      this.clusterEntityIDsList = new ArrayList<List<EntityID>>(
          this.clusterSize );
      
      for ( i = 0; i < this.clusterSize; i++ ) {
        this.clusterEntitiesList.put( i, new ArrayList<StandardEntity>() );
        this.clusterEntityIDsList.add( new ArrayList<EntityID>() );
      }
      
      for ( i = 0; i < mlIndex.length; i++ ) {
        int clusterIndex = Double.valueOf( mlIndex[i] ).intValue() - 1;
        
        EntityID entityId = xyId.get( Double.toString( mlInput[i][0] ) + ","
            + Double.toString( mlInput[i][1] ) );
        
        StandardEntity entity = this.worldInfo.getEntity( entityId );
        
        this.clusterEntitiesList.get( clusterIndex ).add( entity );
      }
      
      for ( i = 0; i < this.clusterEntitiesList.size(); i++ ) {
        for ( StandardEntity entity : this.clusterEntitiesList.get( i ) ) {
          this.clusterEntityIDsList.get( i ).add( entity.getID() );
        }
      }
      
      if ( this.assignAgentsFlag ) {
        List<StandardEntity> firebrigadeList = new ArrayList<StandardEntity>(
            this.worldInfo
                .getEntitiesOfType( StandardEntityURN.FIRE_BRIGADE ) );
        List<StandardEntity> policeforceList = new ArrayList<StandardEntity>(
            this.worldInfo
                .getEntitiesOfType( StandardEntityURN.POLICE_FORCE ) );
        List<StandardEntity> ambulanceteamList = new ArrayList<StandardEntity>(
            this.worldInfo
                .getEntitiesOfType( StandardEntityURN.AMBULANCE_TEAM ) );
        this.assignAgents( firebrigadeList );
        this.assignAgents( policeforceList );
        this.assignAgents( ambulanceteamList );
      }
    } catch ( ExecutionException | InterruptedException e ) {
      e.printStackTrace();
    }
  }
  
  
  private void assignAgents( List<StandardEntity> agentList ) {
    int clusterIndex = 0;
    
    Random r = new Random();
    for ( StandardEntity agent : agentList ) {
      clusterIndex = (int) r.nextInt( this.clusterSize );
      this.clusterEntitiesList.get( clusterIndex ).add( agent );
      this.clusterEntityIDsList.get( clusterIndex ).add( agent.getID() );
    }
  }
  
}