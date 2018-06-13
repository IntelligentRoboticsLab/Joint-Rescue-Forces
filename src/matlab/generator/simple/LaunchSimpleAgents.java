/*******************************************************
 * Simple Agent Launcher
 * 
 * @copyright Copyright 2012 Laboratorio de Tecnicas Inteligentes
 * @author Annibal B. M. da Silva
 * @author Luis G. Nardin
 * @author Jaime S. Sichman
 * @license BSD 3-Clause "New" or "Revised" License
 ********************************************************/
package matlab.generator.simple;

import java.io.IOException;
import sample.SampleCentre;
import matlab.generator.simple.agent.ambulance.SimpleAmbulanceTeam;
import matlab.generator.simple.agent.fire.SimpleFireBrigade;
import matlab.generator.simple.agent.police.SimplePoliceForce;
import rescuecore2.components.ComponentLauncher;
import rescuecore2.components.TCPComponentLauncher;
import rescuecore2.components.ComponentConnectionException;
import rescuecore2.connection.ConnectionException;
import rescuecore2.registry.Registry;
import rescuecore2.misc.CommandLineOptions;
import rescuecore2.config.Config;
import rescuecore2.config.ConfigException;
import rescuecore2.Constants;
import rescuecore2.log.Logger;
import rescuecore2.standard.entities.StandardEntityFactory;
import rescuecore2.standard.entities.StandardPropertyFactory;
import rescuecore2.standard.messages.StandardMessageFactory;

public final class LaunchSimpleAgents {
  
  /**
   * Launch LTI Agent Rescue
   * 
   * @param args
   *          fb fs pf po at ac host port outputDir
   */
  public static void main( String[] args ) {
    Logger.setLogContext( "lti" );
    try {
      Registry.SYSTEM_REGISTRY
          .registerEntityFactory( StandardEntityFactory.INSTANCE );
      Registry.SYSTEM_REGISTRY
          .registerMessageFactory( StandardMessageFactory.INSTANCE );
      Registry.SYSTEM_REGISTRY
          .registerPropertyFactory( StandardPropertyFactory.INSTANCE );
      Config config = new Config();
      args = CommandLineOptions.processArgs( args, config );
      
      int fb = -1;
      int fs = -1;
      int pf = -1;
      int po = -1;
      int at = -1;
      int ac = -1;
      int port = config.getIntValue( Constants.KERNEL_PORT_NUMBER_KEY,
          Constants.DEFAULT_KERNEL_PORT_NUMBER );
      String host = config.getValue( Constants.KERNEL_HOST_NAME_KEY,
          Constants.DEFAULT_KERNEL_HOST_NAME );
      String outputDir = "data";
      
      if ( args.length >= 9 ) {
        fb = Integer.parseInt( args[0] );
        fs = Integer.parseInt( args[1] );
        pf = Integer.parseInt( args[2] );
        po = Integer.parseInt( args[3] );
        at = Integer.parseInt( args[4] );
        ac = Integer.parseInt( args[5] );
        host = args[6];
        port = Integer.parseInt( args[7] );
        outputDir = args[8];
      }
      
      ComponentLauncher launcher = new TCPComponentLauncher( host, port,
          config );
      connect( launcher, fb, fs, pf, po, at, ac, outputDir );
      
    } catch ( IOException e ) {
      Logger.error( "Error connecting agents", e );
    } catch ( ConfigException e ) {
      Logger.error( "Configuration error", e );
    } catch ( ConnectionException e ) {
      Logger.error( "Error connecting agents", e );
    } catch ( InterruptedException e ) {
      Logger.error( "Error connecting agents", e );
    }
    
  }
  
  
  /**
   * 
   * @param launcher
   *          Simulation connection
   * @param fb
   *          fire brigade
   * @param fs
   *          fire station
   * @param pf
   *          police force
   * @param po
   *          police office
   * @param at
   *          ambulance team
   * @param ac
   *          ambulance centre
   * @param outputDir
   *          output directory
   * @throws InterruptedException
   * @throws ConnectionException
   */
  private static void connect( ComponentLauncher launcher, int fb, int fs,
      int pf, int po, int at, int ac, String outputDir )
      throws InterruptedException, ConnectionException {
    
    // Identifies the first launched agent
    boolean first = true;
    
    try {
      int cnt = 1;
      while ( fb-- != 0 ) {
        System.out.print( "Launching Fire Brigade " + cnt + "... " );
        SimpleFireBrigade agent = new SimpleFireBrigade( outputDir );
        launcher.connect( agent );
        if ( first ) {
          agent.logData();
          first = false;
        }
        System.out.println( "success." );
        cnt++;
      }
    } catch ( ComponentConnectionException e ) {
      Logger.info( "failed: " + e.getMessage() );
    }
    System.out.println( "Finished launching Fire Brigades." );
    
    try {
      if ( (fs > 0) || (fs == -1) ) {
        System.out.print( "Launching fire station 1... " );
        launcher.connect( new SampleCentre() );
        System.out.println( "success." );
      }
    } catch ( ComponentConnectionException e ) {
      Logger.info( "failed: " + e.getMessage() );
    }
    System.out.println( "Finished launching Fire Stations." );
    
    Logger.info( "Start launching Police Forces" );
    try {
      int cnt = 1;
      while ( pf-- != 0 ) {
        System.out.print( "Launching Police Force " + cnt + "... " );
        SimplePoliceForce agent = new SimplePoliceForce( outputDir );
        launcher.connect( agent );
        if ( first ) {
          agent.logData();
          first = false;
        }
        System.out.println( "success." );
        cnt++;
      }
    } catch ( ComponentConnectionException e ) {
      Logger.info( "failed: " + e.getMessage() );
    }
    System.out.println( "Finished launching Police Forces." );
    
    try {
      if ( (po > 0) || (po == -1) ) {
        System.out.print( "Launching Police Office 1... " );
        launcher.connect( new SampleCentre() );
        System.out.println( "success." );
      }
    } catch ( ComponentConnectionException e ) {
      Logger.info( "failed: " + e.getMessage() );
    }
    System.out.println( "Finished launching Police Offices." );
    
    try {
      int cnt = 1;
      while ( at-- != 0 ) {
        System.out.print( "Launching Ambulance Team " + cnt + "... " );
        SimpleAmbulanceTeam agent = new SimpleAmbulanceTeam( outputDir );
        launcher.connect( agent );
        if ( first ) {
          agent.logData();
          first = false;
        }
        System.out.println( "success." );
        cnt++;
      }
    } catch ( ComponentConnectionException e ) {
      Logger.info( "failed: " + e.getMessage() );
    }
    System.out.println( "Finished launching Ambulance Teams." );
    
    try {
      if ( (ac > 0) || (ac == -1) ) {
        System.out.print( "Launching Ambulance Centre 1... " );
        launcher.connect( new SampleCentre() );
        System.out.println( "success." );
      }
    } catch ( ComponentConnectionException e ) {
      Logger.info( "failed: " + e.getMessage() );
    }
    System.out.println( "Finished launching Ambulance Centres." );
  }
}
