package nl.tue;

import it.unicam.quasylab.jspear.ControlledSystem;
import it.unicam.quasylab.jspear.DefaultRandomGenerator;
import it.unicam.quasylab.jspear.EvolutionSequence;
import it.unicam.quasylab.jspear.SampleSet;
import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.ControllerRegistry;
import it.unicam.quasylab.jspear.distl.*;
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateFunction;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import it.unicam.quasylab.jspear.udistl.UDisTLFormula;
import it.unicam.quasylab.jspear.udistl.UnboundedUntiluDisTLFormula;
import nl.tue.Monitoring.Default.DefaultMonitorBuilder;
import nl.tue.Monitoring.PerceivedSystemState;
import nl.tue.Monitoring.Default.DefaultUDisTLMonitor;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.List;
import java.util.OptionalDouble;
import java.util.function.Function;

public class Main {

    private static final int ES_SAMPLE_SIZE = 20;
    private static final int MONITORING_SAMPLE_SIZE = 20;

    private static final int t = 0;
    private static final int x = 1;


    public static void main(String[] args) {
        // Set up of evolution sequence. An evolution sequence will be the source of observations.
        Controller controller = getIdleController();

        Function<RandomGenerator, Double> myGaussian = (rg) -> rg.nextGaussian()/3+0.5;

        DataStateFunction environment = (rg, ds) ->
            ds.apply(List.of(
                    new DataStateUpdate(t, ds.get(t)+1),
                    new DataStateUpdate(x, myGaussian.apply(rg)+(1-1/(ds.get(t)+1))))
          );

        EvolutionSequence sequence = new EvolutionSequence(new DefaultRandomGenerator(),
                rg -> new ControlledSystem(controller, environment, new DataState(new double[]{0, myGaussian.apply(rg)})),
                ES_SAMPLE_SIZE);

        // Creating a uDisTL formula
        DataStateFunction mu = (rg, ds) -> ds.apply(List.of(new DataStateUpdate(x, myGaussian.apply(rg))));

        DisTLFormula phiprime = new TargetDisTLFormula(mu, ds -> ds.get(x), 0.0);
        UDisTLFormula phi = new NegationDisTLFormula(new UnboundedUntiluDisTLFormula(new TrueDisTLFormula(),
                new NegationDisTLFormula(phiprime)));

        // Creating a monitor for the formula
        DefaultMonitorBuilder defaultMonitorBuilder = new DefaultMonitorBuilder(MONITORING_SAMPLE_SIZE, false);
        DefaultUDisTLMonitor m = defaultMonitorBuilder.build(phi);

        // Printing the first 10 monitor outputs
        int i = 0;
        while(i < 10){
            SampleSet<PerceivedSystemState> distribution = sequence.getAsPerceivedSystemStates(i);
            OptionalDouble monitorEval = m.evalNext(distribution);
            System.out.println(monitorEval.isPresent() ? monitorEval.getAsDouble() : "u");
            i++;
        }
    }

    public static Controller getIdleController() {
        ControllerRegistry registry = new ControllerRegistry();
        registry.set("Ctrl",
                Controller.doTick(registry.reference("Ctrl"))
        );
        return registry.reference("Ctrl");
    }
}
