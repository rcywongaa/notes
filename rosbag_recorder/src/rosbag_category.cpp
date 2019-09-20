#include <rosbag_recorder/rosbag_category.h>
#include <string>

RosbagCategory::RosbagCategory() {
  this->categoryName = "";
}

RosbagCategory::RosbagCategory(std::string name) {
  this->categoryName = name;
}

RosbagCategory::RosbagCategory(std::string name,
                               std::vector<RosbagTopic> topicList) {
  this->categoryName = name;
  this->toplicList = topicList;
}

RosbagCategory::RosbagCategory(std::string name, std::string topicArray[]) {
  this->categoryName = name;
  int arraySize = topicArray->length();
  for (int index = 0; index < arraySize; index++) {
    std::string topicName = topicArray[index];
    RosbagTopic eachTopic = RosbagTopic(topicName);
    this->toplicList.push_back(eachTopic);
  }
}

RosbagCategory::RosbagCategory(std::string name,
                               std::vector<std::string> topicNameList) {
  this->categoryName = name;
  int listSize = topicNameList.size();
  for (int index = 0; index < listSize; index++) {
    std::string topicName = topicNameList.at(index);
    RosbagTopic eachTopic = RosbagTopic(topicName);
    this->toplicList.push_back(eachTopic);
  }
}
std::string RosbagCategory::toString() {
  //(AT) will this be executed?
  //if (this == NULL)
  //  return "RosbagCategory object is NULL";
  std::string returnedString = "RosbagCaterogy name: " + this->getName() + "\n";
  int listSize = this->toplicList.size();
  returnedString += ("Topic List has following topic(s):\n");
  for (int index = 0; index < listSize; index++) {
    returnedString += "\t" + this->toplicList.at(index).toString() + "\n";
  }
  return returnedString;
}

std::string RosbagCategory::getName() {
  return this->categoryName;
}

std::vector<RosbagTopic> RosbagCategory::getTopicList() {
  return this->toplicList;
}

RosbagTopic::RosbagTopic() {
  this->topicName = "";
}

RosbagTopic::RosbagTopic(std::string name) {
  this->topicName = name;
}

std::string RosbagTopic::getName() {
  return this->topicName;
}

std::string RosbagTopic::toString() {
  return this->topicName;
}
