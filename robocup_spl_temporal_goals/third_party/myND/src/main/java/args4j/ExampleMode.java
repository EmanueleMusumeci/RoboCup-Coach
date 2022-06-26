package args4j;

import args4j.spi.OptionHandler;

/**
 * Used with {@link CmdLineParser#printExample(ExampleMode)}.
 *
 * @author Kohsuke Kawaguchi
 *
 * @deprecated use {@link OptionHandlerFilter}
 */
public enum ExampleMode implements OptionHandlerFilter {
  /**
   * Print all defined options in the example.
   *
   * <p>
   * This would only be useful with small numbers of options.
   */
  ALL() {
    @Override
    public boolean select(OptionHandler o) {
      return true;
    }
  },

  /**
   * Print all {@linkplain Option#required() required} option.
   */
  REQUIRED() {
    @Override
    public boolean select(OptionHandler o) {
      return o.option.required();
    }
  }
}
