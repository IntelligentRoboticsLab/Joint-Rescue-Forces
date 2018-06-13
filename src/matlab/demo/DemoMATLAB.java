package matlab.demo;
/**
 * The program provides a sample for Java Engine functions. Copyright 2016-2017
 * The MathWorks, Inc.
 */

import com.mathworks.engine.MatlabEngine;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;

public class DemoMATLAB {
  
  public static void main( String args[] ) {
    try {
      
      System.out.println( "\nPerforming setup:\n" );
      // Start MATLAB asynchronously
      Future<MatlabEngine> eng = MatlabEngine.startMatlabAsync();
      
      // Get engine instance from the future result
      MatlabEngine ml = eng.get();
      
      /* SETUP */
      // Switch to the right folder
      ml.eval( "cd('tutorial/demo');" );
      // Load the pretrained classifier
      ml.eval( "load myClassifier" );
      
      /* DEFINE TEST DATA */
      double[] mlInput = new double[5];
      mlInput[0] = 25; // Temperature
      mlInput[1] = 30; // Humidity
      mlInput[2] = 300; // Light
      mlInput[3] = 600; // CO2
      mlInput[4] = 0.005; // Humidity ratio
      
      /* PREDICT */
      System.out
          .println( "\nRunning machine learning prediction algorithm:\n" );
      int numSamples = 100;
      for ( int i = 0; i < numSamples; i++ ) {
        
        // Modify light and CO2 values randomly and write "predictors" variable
        // to MATLAB
        mlInput[2] = mlInput[2] + (400 * (Math.random() - 0.5)); // light
        mlInput[3] = mlInput[3] + (1000 * (Math.random() - 0.5)); // CO2
        ml.putVariableAsync( "predictors", mlInput );
        
        // Run the prediction in MATLAB and get "response" variable
        ml.eval( "response = myPredictionFcn(myClassifier,predictors);" );
        Future<Boolean> futureEval = ml.getVariableAsync( "response" );
        Boolean mlOutput = futureEval.get();
        System.out.println( "Predicted occupancy "
            + String.format( "%d", i + 1 ) + " : " + mlOutput );
      }
      
      System.out.println( "\n" );
      
      // Disconnect from the MATLAB session
      ml.disconnect();
    } catch ( ExecutionException | InterruptedException e ) {
      e.printStackTrace();
    }
  }
}
