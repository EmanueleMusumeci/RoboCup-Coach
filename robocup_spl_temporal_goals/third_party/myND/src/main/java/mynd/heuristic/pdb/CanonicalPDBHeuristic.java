package mynd.heuristic.pdb;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import mynd.Global;
import mynd.explicit.ExplicitCondition;
import mynd.heuristic.Heuristic;
import mynd.heuristic.pdb.PatternCollectionSearch.PatternSearch;
import mynd.problem.Problem;
import mynd.state.Condition;
import mynd.state.State;
import mynd.symbolic.BeliefState;
import mynd.symbolic.SymbolicCondition;
import mynd.util.HeuristicValueAggregation;


/**
 *
 * @author Manuela Ortlieb
 * @author Robert Mattmüller
 *
 */
public class CanonicalPDBHeuristic extends Heuristic {

  /**
   * Mapping from the pattern to its pattern database. Only patterns which are used in canonical
   * heuristic function.
   */
  public Map<Set<Integer>, PDB> patterntoPDB;

  /**
   * PDBs only stored temporarily when looking for best successor. After the pattern search these
   * pattern databases could be deleted.
   */
  public Map<Set<Integer>, PDB> temporaryPDBs;

  /**
   * Only one graph is needed for a collection of patterns.
   */
  public CompatibilityGraph compatibilityGraph;

  /**
   * All maximal cliques of a pattern collection.
   */
  public Set<Set<Set<Integer>>> maximalCliques;

  /**
   * Number of abstract states of given pattern collection.
   */
  int size;

  /**
   * Sum of all abstract states in all temporary pdbs.
   */
  public int sizesOfTemporaryPDBs = 0;

  /**
   * Goal condition.
   */
  Condition goal;

  /**
   * Set true for debug output information.
   */
  public static final boolean DEBUG = false;

  /**
   * Constructor
   *
   * @param problem The underlying planning problem
   * @param patternCollection The pattern collection on which this heuristic is based.
   */
  public CanonicalPDBHeuristic(Problem problem, Set<Set<Integer>> patternCollection) {
    super(problem, true); // Canonical PDB-heuristic supports axioms.
    if (PDB.buildExplicitPDBs) {
      goal = problem.explicitGoal;
    } else {
      goal = problem.getGoal();
    }
    initialize(patternCollection);
  }

  public CanonicalPDBHeuristic(Problem problem, Set<Set<Integer>> patternCollection, Condition goal) {
    super(problem, true); // Canonical PDB-heuristic supports axioms.
    this.goal = goal;
    initialize(patternCollection);
  }

  private void initialize(Set<Set<Integer>> patternCollection) {
    if (DEBUG) {
      System.out.println("Constructor of CanonicalPDBHeuristic called.");
    }
    patterntoPDB = new HashMap<Set<Integer>, PDB>();
    temporaryPDBs = new HashMap<Set<Integer>, PDB>();
    fillPDBs(patternCollection);
    compatibilityGraph = new CompatibilityGraph(problem, patternCollection);
    maximalCliques = compatibilityGraph.getMaximalCliques();
    // calculate size
    size = 1;
    for (Set<Integer> pattern : patternCollection) {
      size += PDB.numAbstractStates(problem, pattern);
    }
  }

  public void addPatternToPatternCollection(Set<Integer> newPattern) {
    System.out.println("New Pattern is " + newPattern + ".");
    assert (!patterntoPDB.containsKey(newPattern));
    PDB newPDB;
    if (!Global.options.cachePDBs()) {
      if (problem.isFullObservable || Global.options.assumeFullObservabilityForPDBs()
          || Global.options.patternSearch() == PatternSearch.FO) {
        newPDB = new ExplicitStatePDB(problem, newPattern, (ExplicitCondition) goal);
      } else {
        newPDB = new BeliefStatePDB(problem, newPattern, (SymbolicCondition) goal);
      }
    } else {
      newPDB = temporaryPDBs.get(newPattern);
    }
    patterntoPDB.put(newPattern, newPDB);
    compatibilityGraph.extendCompatibilityGraph(newPattern);
    maximalCliques = compatibilityGraph.getMaximalCliques();
    size += PDB.numAbstractStates(problem, newPattern);
  }

  /**
   * Temporarily add a new PDB for a given pattern.
   */
  public boolean addTemporaryPatternDatabase(Set<Integer> newPattern) {
    assert !patterntoPDB.containsKey(newPattern);
    if (temporaryPDBs.containsKey(newPattern)) {
      return false;
    }
    if (problem.isFullObservable || Global.options.assumeFullObservabilityForPDBs()
        || Global.options.patternSearch() == PatternSearch.FO) {
      temporaryPDBs.put(newPattern, new ExplicitStatePDB(problem, newPattern,
          (ExplicitCondition) goal));
    } else {
      temporaryPDBs.put(newPattern, new BeliefStatePDB(problem, newPattern,
          (SymbolicCondition) goal));
    }
    sizesOfTemporaryPDBs += PDB.numAbstractStates(problem, newPattern);
    return true;
  }

  /**
   * Deletes maximal cliques with dominated sums.
   *
   * Makes sure that <tt>maxcliques</tt> contains the up-to-date maximal cliques.
   */
  public void dominancePruning() {
    System.out.println("Number of maximal cliques before dominance pruning: "
        + maximalCliques.size());

    Set<Set<Set<Integer>>> allMaximalCliques = new HashSet<Set<Set<Integer>>>(maximalCliques);

    nextCandidateDominatedClique: for (Set<Set<Integer>> candidateDominatedClique : allMaximalCliques) {
      for (Set<Set<Integer>> candidateDominatingClique : allMaximalCliques) {
        if (candidateDominatingClique.equals(candidateDominatedClique)) {
          continue;
        }

        boolean allPatternsDominated = true;
        nextCandidateDominatedPattern: for (Set<Integer> candidateDominatedPattern : candidateDominatedClique) {
          for (Set<Integer> candidateDominatingPattern : candidateDominatingClique) {
            if (candidateDominatingPattern.containsAll(candidateDominatedPattern)) {
              continue nextCandidateDominatedPattern;
            }
          }
          allPatternsDominated = false;
          break;
        }
        if (allPatternsDominated) {
          maximalCliques.remove(candidateDominatedClique);
          continue nextCandidateDominatedClique;
        }
      }
    }
    System.out.println("Number of maximal cliques after dominance pruning: "
        + maximalCliques.size());
  }

  private void fillPDB(Set<Integer> pattern) {
    assert (!patterntoPDB.containsKey(pattern));
    if (DEBUG) {
      System.out.println("Pattern added: " + pattern);
    }
    if (PDB.buildExplicitPDBs) {
      patterntoPDB.put(pattern, new ExplicitStatePDB(problem, pattern, (ExplicitCondition) goal));
    } else {
      patterntoPDB.put(pattern, new BeliefStatePDB(problem, pattern, (SymbolicCondition) goal));
    }
  }

  private void fillPDBs(Set<Set<Integer>> patternCollection) {
    if (DEBUG) {
      System.out.println("Fill PDBs.");
    }
    for (Set<Integer> pattern : patternCollection) {
      fillPDB(pattern);
    }
    if (DEBUG) {
      System.out.println("PDBs filled.");
    }
  }

  double getCanonicalHeuristic(State s) {
    double maxH = Double.NEGATIVE_INFINITY;
    for (Set<Set<Integer>> clique : maximalCliques) {
      double current = 0;
      for (Set<Integer> pattern : clique) {
        double patternHeuristicValue = patterntoPDB.get(pattern).getHeuristic(s);
        if (patternHeuristicValue == Heuristic.INFINITE_HEURISTIC) {
          return patternHeuristicValue;
        }
        current += patternHeuristicValue;
      }
      if (current > maxH) {
        maxH = current;
      }
    }
    return maxH;
  }

  /**
   * Get heuristic value for given state.
   *
   * @param state state to be evaluated by heuristic
   * @return heuristic value for given state
   */
  @Override
  public double getHeuristic(State state) {
    if (Global.options.assumeFullObservabilityForPDBs() && state instanceof BeliefState) {
      // In this case we have to draw samples because all PDBs require explicit state.

      // Aggregate heuristic value of given belief state.
      switch (Global.options.getHeuristicAggregationMethod()) {
        case ADD:
          return HeuristicValueAggregation.add(((BeliefState) state)
              .getRandomExplicitWorldStates(Global.options.getNumberOfWorldStatesToBeSampled()),
              this);
        case AVERAGE:
          return HeuristicValueAggregation.average(((BeliefState) state)
              .getRandomExplicitWorldStates(Global.options.getNumberOfWorldStatesToBeSampled()),
              this);
        case MAX:
          return HeuristicValueAggregation.maximize(((BeliefState) state)
              .getRandomExplicitWorldStates(Global.options.getNumberOfWorldStatesToBeSampled()),
              this);
        default:
          assert false;
          return -1;
      }
    }
    return getCanonicalHeuristic(state);
  }

  @Override
  public String toString() {
    StringBuffer buffer = new StringBuffer();
    buffer.append("Patterns: ");
    for (Set<Integer> pattern : patterntoPDB.keySet()) {
      buffer.append("{ ");
      for (int var : pattern) {
        buffer.append(var);
        buffer.append(" ");
      }
      buffer.append("} ");
    }
    buffer.append("\n");
    buffer.append("Maximal cliques: " + maximalCliques);
    return buffer.toString();
  }
}
