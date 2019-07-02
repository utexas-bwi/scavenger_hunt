#include <iostream>
#include <vector>

struct VectorMember {
  int i;
};

struct VectorStruct {
  std::vector<VectorMember> vec;
};

void f(VectorStruct &vs) {
  VectorMember vm;
  vs.vec.push_back(vm);
}

int main(int argc, char **argv) {
  VectorStruct vs;
  f(vs);
  std::cout << vs.vec.size() << std::endl;
  f(vs);
  std::cout << vs.vec.size() << std::endl;
  f(vs);
  std::cout << vs.vec.size() << std::endl;
}
