package args4j;

import java.util.Comparator;

import args4j.spi.OptionHandler;

/**
 * Set of properties that controls {@link CmdLineParser} behaviours.
 *
 * @see CmdLineParser#CmdLineParser(Object, ParserProperties)
 */
public class ParserProperties {

  private static final int DEFAULT_USAGE_WIDTH = 80;

  private int usageWidth = DEFAULT_USAGE_WIDTH;
  private Comparator<OptionHandler> optionSorter = DEFAULT_COMPARATOR;
  private String optionValueDelimiter = " ";

  private ParserProperties() {}

  /**
   * Returns an instance of the default parser properties. This instance can be manipulated with the
   * {@code withXXX()} methods in this class.
   */
  public static ParserProperties defaults() {
    return new ParserProperties();
  }

  /**
   * Sets the width of a usage line. If the usage message is longer than this value, the parser
   * wraps the line.
   *
   * Defaults to {@code 80}.
   *
   * @param usageWidth the width of the usage output in columns.
   * @throws IllegalArgumentException if {@code usageWidth} is negative
   */
  public ParserProperties withUsageWidth(int usageWidth) {
    if (usageWidth < 0) {
      throw new IllegalArgumentException("Usage width is negative");
    }
    this.usageWidth = usageWidth;
    return this;
  }

  /**
   * @return the width of a usage line.
   */
  int getUsageWidth() {
    return usageWidth;
  }

  /**
   * Controls how options are sorted in the usage screen.
   *
   * @param sorter If non-{@code null}, options are sorted in the order induced by this comparator.
   */
  public ParserProperties withOptionSorter(Comparator<OptionHandler> sorter) {
    optionSorter = sorter;
    return this;
  }

  /**
   * @return {@code null} if options are left unsorted and should be listed by their discovery
   *         order. Otherwise the returned comparator is used to sort options. The default value is
   *         a comparator that sorts options alphabetically.
   */
  Comparator<OptionHandler> getOptionSorter() {
    return optionSorter;
  }

  /**
   * Sets the string used to separate option name and its value (such as --foo=bar vs --foo bar)
   *
   * Default to whitespace. Note that the tokens separated in the argument array (such as
   * '-foo','bar') is always recognized as a valid name/value separator.
   *
   */
  public ParserProperties withOptionValueDelimiter(String v) {
    optionValueDelimiter = v;
    return this;
  }

  public String getOptionValueDelimiter() {
    return optionValueDelimiter;
  }

  static final Comparator<OptionHandler> DEFAULT_COMPARATOR = new Comparator<OptionHandler>() {
    @Override
    public int compare(OptionHandler o1, OptionHandler o2) {
      return o1.option.toString().compareTo(o2.option.toString());
    }
  };
}
