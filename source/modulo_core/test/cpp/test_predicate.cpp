#include <gtest/gtest.h>

#include "modulo_core/Predicate.hpp"

using namespace modulo_core;

TEST(PredicateTest, SimplePredicate) {
  auto predicate = Predicate([]() { return true; });
  EXPECT_TRUE(predicate.get_value());
  auto value = predicate.query();
  EXPECT_TRUE(value);
  EXPECT_TRUE(*value);

  EXPECT_TRUE(predicate.get_value());
  value = predicate.query();
  EXPECT_FALSE(value);

  predicate.set_predicate([]() { return false;} );
  EXPECT_FALSE(predicate.get_value());
  value = predicate.query();
  EXPECT_TRUE(value);
  EXPECT_FALSE(*value);

  EXPECT_FALSE(predicate.get_value());
  value = predicate.query();
  EXPECT_FALSE(value);
}
