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

// -----------------  PHASE 2 ------------------------------------------------------------//

TEST(TrojanMapTest, Dijkstra_invalidInput) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  auto path = m.CalculateShortestPath_Dijkstra("asd", "qwe");
  std::vector<std::string> gt;  // expect no path
  std::cout << "My path length: " << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}


TEST(TrojanMapTest, CalculateDijkstra_case1) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Test from Ralphs to Target
  auto path = m.CalculateShortestPath_Dijkstra("Ralphs", "Target");
  std::vector<std::string> gt{
 "2578244375","4380040154","4380040158","4380040167","6805802087","8410938469","6813416131",
 "7645318201","6813416130","6813416129","123318563","452688940","6816193777","123408705","6816193774",
 "452688933","452688931","123230412","6816193770","6787470576","4015442011","6816193692","6816193693",
 "6816193694","4015377691","544693739","6816193696","6804883323","6807937309","6807937306","6816193698",
 "4015377690","4015377689","122814447","6813416159","6813405266","4015372488","4015372487","6813405229",
 "122719216","6813405232","4015372486","7071032399","4015372485","6813379479","6813379584","6814769289","5237417650"} ;
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  // Reverse the input
  path = m.CalculateShortestPath_Dijkstra("Target", "Ralphs");
  std::reverse(gt.begin(),gt.end());
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

TEST(TrojanMapTest, Dijkstra_no_Path) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  auto path = m.CalculateShortestPath_Dijkstra("Starbucks", "University Park");
  std::vector<std::string> gt;  // expect no path
  std::cout << "My path length: " << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  // Reverse the input
  path = m.CalculateShortestPath_Dijkstra("University Park", "Starbucks");
  std::cout << "My path length: " << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

TEST(TrojanMapTest, CalculateDijkstra_case2) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Test from Ralphs to Target
  auto path = m.CalculateShortestPath_Dijkstra("23rd Street Cafe", "Fashion District");
  std::vector<std::string> gt{
"2643391587","6816193725","6042989981","6816193726","6042989994","6042989990","3398621876","3398621867",
"6819317149","6789816350","6816175277","1771004850","6816179435","6816179438","6816179441","6816179439",
"122740185","6816179442","6807274422","1771004841","6816179448","1614922718","1771004866","1832234143",
"1832234142","6816179450","214306755","7053658309","6787830204","6787830208","6225907125","63785537",
"2193435046","1378231766","6208583954","1738419620","1732340071","1732340089","6805686613","1732340094",
"3663661794","1732340079","3663661793","6805686622","6816288743","122740034","6816288741","3396349776",
"7864610979","7864610978","7864621790","7864621789","7864621788","7864621787","7864621786","7864621792",
"7864621791","7864621785","123161956","123250775","3574052741","122956976","6805256746","3574052700",
"4012726941","21098546","1866577831","1738419621","72092084","1738419610","7137906862","4012792184",
"8382592037","122840579","9034507187","9034507186","123120142","4012864456","1788084236","7298792413",
"7298792412","6805885798","122648646","4020262090","4020262091","1924840863","122607549","21098548",
"4020262093","21748269","123120136","4020262096","8178538506","8178538507","8178538508","8178538509",
"8178538510","4010398543"} ;
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  // Reverse the input
  path = m.CalculateShortestPath_Dijkstra("Fashion District", "23rd Street Cafe");
  std::reverse(gt.begin(),gt.end());
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}



TEST(TrojanMapTest, TopologicalSort) {
  TrojanMap m;
  std::vector<std::string> location_names = {"Ralphs", "Chick-fil-A", "KFC"};
  std::vector<std::vector<std::string>> dependencies = {{"Ralphs","KFC"}, {"Ralphs","Chick-fil-A"}, {"KFC","Chick-fil-A"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={"Ralphs", "KFC","Chick-fil-A"};
  EXPECT_EQ(result, gt);
}

TEST(TrojanMapStudentTest, TopologicalSort1) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> location_names = {"The Caribbean Apartments", "Target", "901 Bar & Grill", "Social Security Administration", "Parkside Dining Hall"};
  std::vector<std::vector<std::string>> dependencies = {{"Target","901 Bar & Grill"}, {"Target","Social Security Administration"}, {"The Caribbean Apartments","Social Security Administration"}, {"The Caribbean Apartments","Parkside Dining Hall"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={"Target", "901 Bar & Grill", "The Caribbean Apartments", "Parkside Dining Hall", "Social Security Administration"};
  EXPECT_EQ(result, gt);
}

TEST(TrojanMapStudentTest, TopologicalSort2) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> location_names = {"Food 4 Less", "Ralphs", "Adams-Normandie", "Chevron 2"};
  std::vector<std::vector<std::string>> dependencies = {{"Food 4 Less","Ralphs"}, {"Food 4 Less","Adams-Normandie"}, {"Food 4 Less","Chevron 2"}, {"Chevron 2","Ralphs"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={"Food 4 Less", "Chevron 2", "Adams-Normandie", "Ralphs"}; 

  EXPECT_EQ(result, gt);
}

TEST(TrojanMapStudentTest, TopologicalSort3) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> location_names = {"Ralphs", "Target", "Chipotle Mexican Grill", "CVS", "ChickfilA"};
  std::vector<std::vector<std::string>> dependencies = {{"Target","Chipotle Mexican Grill"}, {"Target","CVS"}, {"Ralphs","CVS"}, {"Ralphs","ChickfilA"}, {"CVS", "Target"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={};
  EXPECT_EQ(result, gt);
}