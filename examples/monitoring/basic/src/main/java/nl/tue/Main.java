package nl.tue;

import it.unicam.quasylab.jspear.ControlledSystem;
import it.unicam.quasylab.jspear.DefaultRandomGenerator;
import it.unicam.quasylab.jspear.EvolutionSequence;
import it.unicam.quasylab.jspear.SampleSet;
import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.ControllerRegistry;
import it.unicam.quasylab.jspear.distl.DisTLFormula;
import it.unicam.quasylab.jspear.distl.DoubleSemanticsVisitor;
import it.unicam.quasylab.jspear.distl.TargetDisTLFormula;
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateFunction;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import it.unicam.quasylab.jspear.udistl.UDisTLFormula;
import nl.tue.Monitoring.DefaultMonitorBuilder;
import nl.tue.Monitoring.PerceivedSystemState;
import nl.tue.Monitoring.UDisTLMonitor;

import java.util.List;

public class Main {

    private static final int ES_SAMPLE_SIZE = 10;
    private static final int FORMULA_SAMPLE_SIZE = 10;
    private static final int MONITORING_SAMPLE_SIZE = 10;

    private static final int x = 0;
    private static final int NUMBER_OF_VARIABLES = 1;

    private static int evaluationTimestep = 0;

    public static void main(String[] args) {
            Controller controller = getController();
            DataStateFunction environment = (rg, ds) -> ds.apply(List.of(new DataStateUpdate(x, rg.nextDouble())));
            DataState initialState = new DataState(NUMBER_OF_VARIABLES, i -> 0.0);
            ControlledSystem system = new ControlledSystem(controller, environment, initialState);
            EvolutionSequence sequence = new EvolutionSequence(new DefaultRandomGenerator(), rg -> system, ES_SAMPLE_SIZE);

            DataStateFunction mu = (rg, ds) -> ds.apply(List.of(new DataStateUpdate(x, 0.0)));

            UDisTLFormula phi = new TargetDisTLFormula(mu, ds -> ds.get(x), 0.0);

            double eval = new DoubleSemanticsVisitor().eval((DisTLFormula) phi)
                    .eval(FORMULA_SAMPLE_SIZE, evaluationTimestep, sequence);

            System.out.println("Robustness: "+eval);


            DefaultMonitorBuilder defaultMonitorBuilder = new DefaultMonitorBuilder(MONITORING_SAMPLE_SIZE, false);
            UDisTLMonitor<Double> m = defaultMonitorBuilder.build(phi, evaluationTimestep);

            SampleSet<PerceivedSystemState> distribution = UDisTLMonitor.systemStatesToPerceivedSystemStates(sequence.get(0));
            double monitorEval = m.evalNext(distribution);
            System.out.println("Monitor eval: "+ monitorEval);

    }

    public static Controller getController() {
        ControllerRegistry registry = new ControllerRegistry();
        registry.set("Ctrl",
                Controller.doAction((rg, ds) -> List.of(new DataStateUpdate(x, rg.nextDouble())), registry.reference("Ctrl"))
        );
        return registry.reference("Ctrl");
    }
}
