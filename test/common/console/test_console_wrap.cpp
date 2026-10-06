/*!
 * @file            test_console_wrap.cpp
 *
 * @brief           Tests the 40-character console wrapping, formatting and
 *                  per-call-site throttling behind the SRS_LOG_* macros.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "console/console.h"

namespace
{

namespace console = common::console;

/*!
 * @brief           Asserts every line respects the limit, so a failure
 *                  names the offending line.
 */
void expectLinesWithin(const std::vector<std::string> &lines_in,
                       std::size_t                     limit_in)
{
    for (const std::string &line : lines_in)
    {
        EXPECT_LE(line.size(), limit_in) << "line: '" << line << "'";
    }
}

TEST(ConsoleWrap, ShortTextIsOneUnchangedLine)
{
    const std::vector<std::string> lines =
        console::wrapText("READY: commands enabled", 40U);
    ASSERT_EQ(lines.size(), 1U);
    EXPECT_EQ(lines[0], "READY: commands enabled");
}

TEST(ConsoleWrap, ExactlyFortyCharactersStaysOnOneLine)
{
    const std::string              text(40U, 'x');
    const std::vector<std::string> lines = console::wrapText(text, 40U);
    ASSERT_EQ(lines.size(), 1U);
    EXPECT_EQ(lines[0], text);
}

TEST(ConsoleWrap, LongTextWrapsAtWordsWithIndentedContinuation)
{
    const std::vector<std::string> lines = console::wrapText(
        "Wheel solve rejected amplified lateral geometry; using "
        "longitudinal/yaw fallback",
        40U);
    ASSERT_EQ(lines.size(), 3U);
    EXPECT_EQ(lines[0], "Wheel solve rejected amplified lateral");
    EXPECT_EQ(lines[1], "  geometry; using longitudinal/yaw");
    EXPECT_EQ(lines[2], "  fallback");
    expectLinesWithin(lines, 40U);
}

TEST(ConsoleWrap, OverlongWordIsSplitAcrossLines)
{
    const std::string path =
        "/alpha/localisation/kalman_filter/odometry_with_a_long_suffix";
    const std::vector<std::string> lines = console::wrapText(path, 40U);
    ASSERT_EQ(lines.size(), 2U);
    EXPECT_EQ(lines[0], path.substr(0U, 40U));
    EXPECT_EQ(lines[1], "  " + path.substr(40U));
    expectLinesWithin(lines, 40U);
}

TEST(ConsoleWrap, NewlinesStartUnindentedLinesAndBlankOnesAreDropped)
{
    const std::vector<std::string> lines =
        console::wrapText("first line\n\nsecond  \t line\n", 40U);
    ASSERT_EQ(lines.size(), 2U);
    EXPECT_EQ(lines[0], "first line");
    EXPECT_EQ(lines[1], "second line");
}

TEST(ConsoleWrap, EmptyTextIsOneEmptyLine)
{
    const std::vector<std::string> lines = console::wrapText("   ", 40U);
    ASSERT_EQ(lines.size(), 1U);
    EXPECT_TRUE(lines[0].empty());
}

TEST(ConsoleWrap, TinyLimitIsRaisedToThree)
{
    const std::vector<std::string> lines = console::wrapText("abcdef", 1U);
    ASSERT_EQ(lines.size(), 4U);
    EXPECT_EQ(lines[0], "abc");
    EXPECT_EQ(lines[1], "  d");
    expectLinesWithin(lines, 3U);
}

TEST(ConsoleWrap, EveryLineOfLongMixedTextFits)
{
    std::string text;
    for (int index = 0; index < 60; ++index)
    {
        text += "token" + std::to_string(index * 7919) + " ";
    }
    const std::vector<std::string> lines = console::wrapText(text, 40U);
    EXPECT_GT(lines.size(), 10U);
    expectLinesWithin(lines, 40U);
}

TEST(ConsoleFormat, FormatsLikePrintf)
{
    EXPECT_EQ(console::formatText("VO %.1fHz inl %d fail %zu",
                                  1.94,
                                  180,
                                  static_cast<std::size_t>(0U)),
              "VO 1.9Hz inl 180 fail 0");
    const std::string longText(500U, 'y');
    EXPECT_EQ(console::formatText("%s", longText.c_str()), longText);
}

TEST(ConsoleThrottle, FirstMessageAlwaysEmits)
{
    console::ThrottleGate gate;
    EXPECT_TRUE(gate.shouldEmit(0, 3000000000));
}

TEST(ConsoleThrottle, SuppressesWithinThePeriod)
{
    console::ThrottleGate gate;
    ASSERT_TRUE(gate.shouldEmit(10000000000, 3000000000));
    EXPECT_FALSE(gate.shouldEmit(12999999999, 3000000000));
    EXPECT_TRUE(gate.shouldEmit(13000000000, 3000000000));
    EXPECT_FALSE(gate.shouldEmit(13000000001, 3000000000));
}

TEST(ConsoleThrottle, BackwardsClockJumpRestartsThePeriod)
{
    console::ThrottleGate gate;
    ASSERT_TRUE(gate.shouldEmit(50000000000, 3000000000));
    EXPECT_TRUE(gate.shouldEmit(1000000000, 3000000000));
    EXPECT_FALSE(gate.shouldEmit(2000000000, 3000000000));
}

TEST(ConsoleThrottle, NonPositivePeriodAdmitsEveryMessage)
{
    console::ThrottleGate gate;
    EXPECT_TRUE(gate.shouldEmit(5, 0));
    EXPECT_TRUE(gate.shouldEmit(5, 0));
}

} /* namespace */
