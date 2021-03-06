#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <scavenger_hunt/rapidxml.hpp>
#include <scavenger_hunt/scavenger_hunt_client.h>
#include <stdio.h>
#include <string>
#include <string.h>

using namespace rapidxml;

static const std::string DOWNLOAD_URL = "/script/get_tasks.php";
static const std::string UPLOAD_URL = "/script/upload_proof.php";
static const std::string PROOFS_URL = "/script/get_proofs.php";
static const std::string PROOF_MATERIALS_URL = "/proof";
static const std::string GET_PROOF_MATERIAL_URL = "/script/get_proof_material_url.php";
static const std::string PROOF_STATUS_URL = "/script/get_proof_status.php";

static std::string user_email;
static int user_password_hash;

namespace {

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
    @brief cb for cURL to write POST data into a file
  */
  std::size_t curl_write_file_cb(void *ptr, std::size_t size, std::size_t num,
      FILE *stream) {
    std::size_t written = fwrite(ptr, size, num, stream);
    return written;
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
    @brief parses task info from website XML
  */
  void parse_hunt_xml(std::string *xml, std::vector<Task> &tasks) {
    xml_document<> doc;
    xml_node<> *root_node;

    char *buffer = new char[xml->size() + 1];
    strcpy(buffer, xml->c_str());

    doc.parse<0>(buffer);
    root_node = doc.first_node("hunt");
    std::string hunt_name = root_node->first_attribute("name")->value();

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

      Task task(name, hunt_name, description, proof_format, proof_description,
          points, id);

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

  /**
    @brief parses proof info from website XML
  */
  void parse_proof_xml(std::string *xml, std::vector<Proof> &proofs) {
    xml_document<> doc;
    xml_node<> *root_node;

    char *buffer = new char[xml->size() + 1];
    strcpy(buffer, xml->c_str());

    doc.parse<0>(buffer);
    root_node = doc.first_node("task");

    if (root_node == nullptr)
      return;

    for (xml_node<> *task_node = root_node->first_node("proof");
         task_node;
         task_node = task_node->next_sibling()) {
      bool correct = std::string(task_node->first_attribute("correct")->value()) == "0" ? false : true;
      std::string url = std::string(task_node->first_attribute("filename")->value());
      int time_to_complete = std::stoi(std::string(task_node->first_attribute("time")->value()));
      proof_id_t id = std::stoi(std::string(task_node->first_attribute("id")->value()));

      Proof proof(correct, time_to_complete, url, id);
      proofs.push_back(proof);
    }

    delete buffer;
  }

  /**
    @brief download a file from a URL and put it somewhere on disk
  */
  bool curl_download_file(std::string url, std::string destination) {
    CURL *curl = curl_easy_init();

    FILE *fout = fopen(destination.c_str(), "wb");

    // Do cURL request
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10); // Time out after 10 seconds
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L); // Allow 1 redirect
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write_file_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, fout);

    curl_easy_perform(curl);

    int http_response_code;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_response_code);

    if (http_response_code != 200)
      return false;

    curl_easy_cleanup(curl);
    fclose(fout);

    return true;
  }

} // private namespace

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

  curl_easy_setopt(curl, CURLOPT_URL, (domain + DOWNLOAD_URL).c_str());
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

void ScavengerHuntClient::get_proofs(Task &task, std::vector<Proof> &proofs) {
  // Configure cURL request
  CURL *curl = curl_easy_init();
  std::string http_received_data;

  curl_easy_setopt(curl, CURLOPT_URL, PROOFS_URL.c_str());
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10); // Time out after 10 seconds
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L); // Allow 1 redirect
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,
      curl_write_cb);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &http_received_data);

  struct curl_httppost *post_begin = NULL;
	struct curl_httppost *post_end = NULL;

  // Get task fields
  std::string parameter;

  for (const auto &pair : task.get_parameters())
    parameter = pair.second;

  std::string hunt_name = task.get_hunt_name();
  std::string task_name = task.get_name();

  curl_formadd(&post_begin,
               &post_end,
             	 CURLFORM_COPYNAME, "hunt_name",
             	 CURLFORM_COPYCONTENTS, hunt_name.c_str(),
             	 CURLFORM_END);
  curl_formadd(&post_begin,
               &post_end,
               CURLFORM_COPYNAME, "task_name",
               CURLFORM_COPYCONTENTS, task_name.c_str(),
               CURLFORM_END);
  curl_formadd(&post_begin,
               &post_end,
               CURLFORM_COPYNAME, "param",
               CURLFORM_COPYCONTENTS, parameter.c_str(),
               CURLFORM_END);
  curl_formadd(&post_begin,
               &post_end,
               CURLFORM_COPYNAME, "user_email",
               CURLFORM_COPYCONTENTS, user_email.c_str(),
               CURLFORM_END);
  curl_easy_setopt(curl, CURLOPT_POST, true);
  curl_easy_setopt(curl, CURLOPT_HTTPPOST, post_begin);

  // Fetch website response
  int http_response_code = 0;

  curl_easy_perform(curl);

  if (http_received_data.length() > 0) {
    // Response good
    std::cout << get_telemetry_tag(user_email, "get_proofs") <<
        "Got response from Scavenger Hunt server. Parsing..." << std::endl;

    parse_proof_xml(&http_received_data, proofs);

    if (proofs.size() > 0)
      std::cout << get_telemetry_tag(user_email, "get_proofs") <<
          "Successfully parsed " << proofs.size() << " proofs(s)." << std::endl;
    else
      std::cout << get_telemetry_tag(user_email, "get_proofs") <<
          "No proofs found." << std::endl;
  } else {
    // Couldn't contact website
    std::cout << get_telemetry_tag(user_email, "get_proofs") <<
        "Failed to contact Scavenger Hunt." << std::endl;
  }

  // Cleanup
  curl_easy_cleanup(curl);
}

void ScavengerHuntClient::download_proof_material(Proof &proof,
    std::string filepath) {
  std::cout << get_telemetry_tag(user_email, "download_proof_material") <<
      "Preparing to download proof material..." << std::endl;

  // Ensure path is valid
  if (!file_exists(filepath)) {
    std::cout << get_telemetry_tag(user_email, "download_proof_material") <<
        "Specified path is invalid: " << filepath << std::endl;
    return;
  }

  std::string http_received_data;
  std::string url = domain + PROOF_MATERIALS_URL + "/" + proof.get_filename();
  std::string dest = filepath + "/" + proof.get_filename();

  bool success = curl_download_file(url, dest);

  if (success)
    std::cout << get_telemetry_tag(user_email, "download_proof_material") <<
        "Download successful." << std::endl;
  else
    std::cout << get_telemetry_tag(user_email, "download_proof_material") <<
        "Failed to contact Scavenger Hunt." << std::endl;
}

void ScavengerHuntClient::download_proof_material(proof_id_t id,
    std::string filepath) {
  std::cout << get_telemetry_tag(user_email, "download_proof_material") <<
      "Preparing to download proof material..." << std::endl;

  CURL *curl = curl_easy_init();
  std::string http_received_data;

  // Do cURL request
  struct curl_httppost *post_begin = NULL;
	struct curl_httppost *post_end = NULL;

  std::string id_str = std::to_string(id);

  curl_formadd(&post_begin,
               &post_end,
             	 CURLFORM_COPYNAME, "id",
             	 CURLFORM_COPYCONTENTS, id_str.c_str(),
             	 CURLFORM_END);
  curl_easy_setopt(curl, CURLOPT_URL,
                   (domain + GET_PROOF_MATERIAL_URL).c_str());
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10); // Time out after 10 seconds
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L); // Allow 1 redirect
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write_cb);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &http_received_data);
  curl_easy_setopt(curl, CURLOPT_POST, true);
  curl_easy_setopt(curl, CURLOPT_HTTPPOST, post_begin);

  curl_easy_perform(curl);

  int http_response_code;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_response_code);
  curl_easy_cleanup(curl);

  if (http_response_code == 200) {
    std::string url = domain + PROOF_MATERIALS_URL + "/" + http_received_data;
    bool success = curl_download_file(url, filepath);

    if (success)
      std::cout << get_telemetry_tag(user_email, "download_proof_material") <<
          "Download successful." << std::endl;
    else
      std::cout << get_telemetry_tag(user_email, "download_proof_material") <<
          "Failed to contact Scavenger Hunt." << std::endl;
  } else
    std::cout << get_telemetry_tag(user_email, "download_proof_material") <<
        "Failed to contact Scavenger Hunt." << std::endl;
}

proof_status_t ScavengerHuntClient::get_proof_status(proof_id_t id) {
  std::cout << get_telemetry_tag(user_email, "get_proof_status") <<
      "Querying server for status of proof " << id << "..." << std::endl;

  CURL *curl = curl_easy_init();
  std::string http_received_data;

  // Do cURL request
  curl_easy_setopt(curl, CURLOPT_URL, (domain + PROOF_STATUS_URL).c_str());
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10); // Time out after 10 seconds
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L); // Allow 1 redirect
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write_cb);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &http_received_data);

  struct curl_httppost *post_begin = NULL;
	struct curl_httppost *post_end = NULL;

  std::string id_str = std::to_string(id);

  curl_formadd(&post_begin,
               &post_end,
             	 CURLFORM_COPYNAME, "id",
             	 CURLFORM_COPYCONTENTS, id_str.c_str(),
             	 CURLFORM_END);
 curl_easy_setopt(curl, CURLOPT_POST, true);
 curl_easy_setopt(curl, CURLOPT_HTTPPOST, post_begin);

  curl_easy_perform(curl);

  int http_response_code;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_response_code);

  proof_status_t status = PROOF_NOT_VALIDATED;

  if (http_response_code == 200) {
    std::cout << get_telemetry_tag(user_email, "get_proof_status") <<
        "Query successful." << std::endl;

    int status_code = std::stoi(http_received_data);

    switch (status_code) {
      case 0:
        status = PROOF_INCORRECT;
        break;

      case 1:
        status = PROOF_CORRECT;
        break;
    }

  } else
    std::cout << get_telemetry_tag(user_email, "get_proof_status") <<
        "Failed to contact Scavenger Hunt." << std::endl;

  curl_easy_cleanup(curl);

  return status;
}

proof_id_t ScavengerHuntClient::send_proof(std::string file_path,
                                           Task &task,
                                           double time)
{
  return send_proof(file_path, task.get_hunt_task_id(), time);
}

proof_id_t ScavengerHuntClient::send_proof(std::string image_path,
                                           int hunt_task_id,
                                           double time)
{
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
  curl_easy_setopt(curl, CURLOPT_URL, (domain + UPLOAD_URL).c_str());

  struct curl_httppost *post_begin = NULL;
	struct curl_httppost *post_end = NULL;

  std::string password_hash_str = std::to_string(user_password_hash);
  std::string instruction_id_str = std::to_string(hunt_task_id);
  std::string time_str = std::to_string(time);
  std::string http_received_data;

  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,
      curl_write_cb);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &http_received_data);

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

  proof_id_t uid = UPLOAD_FAILED;

  std::cout << http_received_data;

  bool upload_succeeded =
      http_received_data.find("failed") == std::string::npos;

  if (!success)
    std::cout << get_telemetry_tag(user_email, "send_proof") <<
        "Failed to contact Scavenger Hunt." << std::endl;
  else if (upload_succeeded) {
    // Parse proof ID
    std::string key = "Proof ID: ";
    std::size_t pos = http_received_data.find(key);
    std::size_t id_start = pos + key.length();
    std::string sub = http_received_data.substr(id_start,
        http_received_data.length() - id_start);
    uid = std::stoi(sub);
  }

  return uid;
}
