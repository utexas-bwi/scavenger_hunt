#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <scavenger_hunt/rapidxml.hpp>
#include <scavenger_hunt/scavenger_hunt_client.h>
#include <string>
#include <string.h>

using namespace rapidxml;

static const char DOWNLOAD_URL[] = "localhost/script/get_tasks.php";
static const char UPLOAD_URL[] = "localhost/script/upload_proof.php";

static std::string user_email;
static int user_password_hash;

/**
  @brief callback function used by cURL to offload buffered POST data
*/
std::size_t curl_write_cb(const char *in, std::size_t size,
    std::size_t num, std::string *out) {
  const std::size_t total_bytes(size * num);
  out->append(in, total_bytes);
  return total_bytes;
}

/**
  Java's 32-bit string hash algorithm. The website uses the same algorithm
  to hash passwords and user IDs.

  @param str string to hash
  @return hash
*/
int strhash32(std::string str) {
	int hash = 0;
	if (str.length() == 0)
		return hash;
	const char *str_arr = str.c_str();
	for (int i = 0; i < str.length(); i++) {
		char c = str_arr[i];
		hash = ((hash << 5) - hash) + c;
	}
	return hash;
}

/**
  @brief gets if the file at path fname exists
*/
bool file_exists(std::string fname) {
  std::ifstream f(fname.c_str());
  return f.good();
}

/**
  @brief parses a ScavengerHunt object from XML text
*/
void parse_hunt_xml(std::string *xml, std::vector<Task> &tasks) {
  xml_document<> doc;
  xml_node<> *root_node;

  char *buffer = new char[xml->size() + 1];
  strcpy(buffer, xml->c_str());

  doc.parse<0>(buffer);
  root_node = doc.first_node("hunt");

  if (root_node == nullptr)
    return;

  for (xml_node<> *task_node = root_node->first_node("task");
       task_node;
       task_node = task_node->next_sibling()) {
    // Parse task fields
    std::string name = std::string(task_node->first_attribute("name")->value());
    std::string description = std::string(task_node->first_attribute("description")->value());
    std::string proof_format = std::string(task_node->first_attribute("proof_format")->value());
    std::string proof_description = std::string(task_node->first_attribute("proof_description")->value());
    int points = std::stoi(std::string(task_node->first_attribute("points")->value()));
    int id = std::stoi(std::string(task_node->first_attribute("id")->value()));

    Task task(name, description, proof_format, proof_description, points, id);

    // Parse task parameters
    for (xml_node<> *param_node = task_node->first_node("parameter");
         param_node;
         param_node = param_node->next_sibling()) {
      std::string param_name = std::string(param_node->first_attribute("name")->value());
      std::string param_value = std::string(param_node->first_attribute("value")->value());
      task.add_parameter(param_name, param_value);
    }

    tasks.push_back(task);
  }

  delete buffer;
}

ScavengerHuntClient::ScavengerHuntClient(std::string email,
    std::string password) {
  user_email = email;
  user_password_hash = strhash32(password);
}

std::string get_telemetry_tag(std::string user_email,
    std::string method_name) {
  return "[" + user_email + "//" + method_name + "] ";
}

void ScavengerHuntClient::get_hunt(std::string hunt_name,
    std::vector<Task> &tasks) {
  // Configure cURL request
  CURL *curl = curl_easy_init();
  std::string http_received_data;

  curl_easy_setopt(curl, CURLOPT_URL, DOWNLOAD_URL);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10); // Time out after 10 seconds
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L); // Allow 1 redirect
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,
      curl_write_cb);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &http_received_data);

  struct curl_httppost *post_begin = NULL;
	struct curl_httppost *post_end = NULL;

  curl_formadd(&post_begin,
               &post_end,
             	 CURLFORM_COPYNAME, "hunt_name",
             	 CURLFORM_COPYCONTENTS, hunt_name.c_str(),
             	 CURLFORM_END);
  curl_easy_setopt(curl, CURLOPT_POST, true);
  curl_easy_setopt(curl, CURLOPT_HTTPPOST, post_begin);

  // Fetch website response
  int http_response_code = 0;

  curl_easy_perform(curl);

  if (http_received_data.length() > 0) {
    // Response good
    std::cout << get_telemetry_tag(user_email, "get_hunt") <<
        "Got response from Scavenger Hunt server. Parsing..." << std::endl;

    parse_hunt_xml(&http_received_data, tasks);

    if (tasks.size() > 0)
      std::cout << get_telemetry_tag(user_email, "get_hunt") <<
          "Successfully parsed " << tasks.size() << " task(s)." << std::endl;
    else
      std::cout << get_telemetry_tag(user_email, "get_hunt") <<
          "Could not find hunt with name \"" << hunt_name << "\"." << std::endl;
  } else {
    // Couldn't contact website
    std::cout << get_telemetry_tag(user_email, "get_hunt") <<
        "Failed to contact Scavenger Hunt." << std::endl;
  }

  // Cleanup
  curl_easy_cleanup(curl);
}

bool ScavengerHuntClient::send_proof(std::string image_path, Task &task, double time) {
  std::cout << get_telemetry_tag(user_email, "send_proof") <<
      "Preparing to send proof..." << std::endl;

  // Ensure path is valid
  if (!file_exists(image_path)) {
    std::cout << get_telemetry_tag(user_email, "send_proof") <<
        "Could not find file \"" << image_path << "\"." << std::endl;
    return false;
  }

  // Configure cURL request
  CURL *curl = curl_easy_init();
  curl_easy_setopt(curl, CURLOPT_URL, UPLOAD_URL);

  struct curl_httppost *post_begin = NULL;
	struct curl_httppost *post_end = NULL;

  std::string password_hash_str = std::to_string(user_password_hash);
  std::string instruction_id_str = std::to_string(task.get_hunt_task_id());
  std::string time_str = std::to_string(time);

  // Create upload form
  curl_formadd(&post_begin,
	             &post_end,
							 CURLFORM_COPYNAME, "image",
               CURLFORM_FILE, image_path.c_str(),
							 CURLFORM_END);
	curl_formadd(&post_begin,
               &post_end,
             	 CURLFORM_COPYNAME, "email",
             	 CURLFORM_COPYCONTENTS, user_email.c_str(),
             	 CURLFORM_END);
	curl_formadd(&post_begin,
	             &post_end,
	          	 CURLFORM_COPYNAME, "pass_hash",
	          	 CURLFORM_COPYCONTENTS, password_hash_str.c_str(),
	          	 CURLFORM_END);
  curl_formadd(&post_begin,
						   &post_end,
						   CURLFORM_COPYNAME, "instr_id",
						   CURLFORM_COPYCONTENTS, instruction_id_str.c_str(),
						   CURLFORM_END);
  curl_formadd(&post_begin,
              &post_end,
              CURLFORM_COPYNAME, "time",
              CURLFORM_COPYCONTENTS, time_str.c_str(),
              CURLFORM_END);

  // Perform cURL
  curl_easy_setopt(curl, CURLOPT_POST, true);
	curl_easy_setopt(curl, CURLOPT_HTTPPOST, post_begin);
	CURLcode code = curl_easy_perform(curl);

  int http_response_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_response_code);
  bool success = http_response_code == 200;

  // Cleanup
  curl_easy_cleanup(curl);

  std::cout << get_telemetry_tag(user_email, "send_proof") <<
      "Proof uploaded!" << std::endl;

  return success;
}
