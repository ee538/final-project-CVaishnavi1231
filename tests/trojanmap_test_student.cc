#include "gtest/gtest.h"
#include "src/lib/trojanmap.h"

#include "gtest/gtest.h"
#include "src/lib/trojanmap.h"

TEST(TrojanMapStudentTest, GetLat) {
  TrojanMap m;
  EXPECT_EQ(m.GetLat("653725"), 34.0360852);
}

TEST(TrojanMapStudentTest, GetName) {
  TrojanMap m;
  EXPECT_EQ(m.GetName("122659207"), "Crosswalk");
}


TEST(TrojanMapStudentTest, GetID1) {
  TrojanMap m;
  EXPECT_EQ(m.GetID("West Side Church of God"), "358850081");
}

TEST(TrojanMapStudentTest, GetID2) {
  TrojanMap m;
  EXPECT_EQ(m.GetID("West Side Church"), "");
}

TEST(TrojanMapStudentTest, GetLon) {
  TrojanMap TM;
  EXPECT_EQ(TM.GetLon("653725"), -118.3212048);
}

TEST(TrojanMapStudentTest, GetNeighborIDs) {
  TrojanMap TM;
  std::vector<std::string>  out = {"277327731", "1613324102"};
  EXPECT_EQ(TM.GetNeighborIDs("653725"), out);
}

TEST(TrojanMapStudentTest, GetPosition) {
  TrojanMap TM;
  std::pair<double, double> out(34.0302951, -118.2857237);
  EXPECT_EQ(TM.GetPosition("Crosswalk"), out);
}

TEST(TrojanMapTest, FindClosestName) {
  TrojanMap TM;
  EXPECT_EQ(TM.FindClosestName("Rolphs"), "Ralphs");
  EXPECT_EQ(TM.FindClosestName("Targeety"), "Target");
}

// ---------------- PHASE-1 TESTS --------------------------------------------


TEST(TrojanMapTest, Autocomplete) {
  TrojanMap m;
  // Test the simple case
  auto names = m.Autocomplete("Chi");
  std::unordered_set<std::string> gt = {"Chick-fil-A", "Chipotle", "Chinese Street Food"}; // groundtruth for "Ch"
  int success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower case
  names = m.Autocomplete("chi");
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower and upper case 
  names = m.Autocomplete("cHi"); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the upper case 
  names = m.Autocomplete("CHI"); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
}

// Test CalculateEditDistance function
TEST(TrojanMapTest, CalculateEditDistance) {
  TrojanMap m;
  EXPECT_EQ(m.CalculateEditDistance("horse", "ros"), 3);
  EXPECT_EQ(m.CalculateEditDistance("intention", "execution"), 5);
}


// Test FindPosition function
TEST(TrojanMapTest, FindPosition) {
  TrojanMap m;
  
  // Test Chick-fil-A
  auto position = m.GetPosition("Chick-fil-A");
  std::pair<double, double> gt1(34.0167334, -118.2825307); // groundtruth for "Chick-fil-A"
  EXPECT_EQ(position, gt1);
  // Test Ralphs
  position = m.GetPosition("Ralphs");
  std::pair<double, double> gt2(34.0317653, -118.2908339); // groundtruth for "Ralphs"
  EXPECT_EQ(position, gt2);
  // Test Target
  position = m.GetPosition("Target");
  std::pair<double, double> gt3(34.0257016, -118.2843512); // groundtruth for "Target"
  EXPECT_EQ(position, gt3);
  // Test Unknown
  position = m.GetPosition("XXX");
  std::pair<double, double> gt4(-1, -1);
  EXPECT_EQ(position, gt4);
}