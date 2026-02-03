// Copyright 2024 Blazej Fiderek (xfiderek)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <sstream>
#include <string>
#include <vector>

double trick_string_convert(const std::string& str)
{
  std::istringstream ss(str);
  double num;
  ss >> num;
  return num;
}

std::vector<std::string> trick_split_response(std::string& str, const char delim)
{
  std::stringstream ss(str);
  std::string s;
  std::vector<std::string> ret;
  while (ss >> s)
  {
    ret.push_back(s);
  }
  return ret;
}

std::vector<double> trick_response_convert(std::string& response)
{
  auto responseSplit = trick_split_response(response, '\t');
  std::vector<double> result;
  if (responseSplit[0] != "0")
  {
    return result;
  }
  for (int i = 1; i < responseSplit.size(); i++)
  {
    result.push_back(trick_string_convert(responseSplit[i]));
  }
  return result;
}
