#include "scavenger/scavenger_hunt.h"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

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

ScavengerHuntClient::ScavengerHuntClient(std::string email,
    std::string password) {
  user_email = email;
  user_password_hash = strhash32(password);
}

void ScavengerHuntClient::get_hunts() {
  // Configure cURL request
  CURL *curl = curl_easy_init();
  std::string http_received_data;

  curl_easy_setopt(curl, CURLOPT_URL, DOWNLOAD_URL);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10); // Time out after 10 seconds
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L); // Allow 1 redirect
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,
      curl_write_cb);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &http_received_data);

  // Fetch website response
  int http_response_code = 0;

  curl_easy_perform(curl);
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_response_code);

  if (http_response_code == 200) {
    // Response good
    std::cout << "[retrieve] Got response from Scavenger Hunt server. Parsing..." << std::endl;

    std::stringstream parser_stream;
    parser_stream << http_received_data.c_str();

    Json::Value json_data;
    Json::CharReaderBuilder json_reader;
    std::string errs;

    Json::parseFromStream(json_reader, parser_stream, &json_data, &errs);

    std::cout << "[retrieve] Successfully parsed task data." << std::endl;
  } else {
    // Couldn't contact website
    std::cout << "[retrieve] Failed to contact Scavenger Hunt server." << std::endl;
  }

  // Cleanup
  curl_easy_cleanup(curl);
}

bool ScavengerHuntClient::send_proof(std::string image_path,
    unsigned int instruction_id) {
  std::cout << "[send_proof] Preparing to send proof..." << std::endl;

  // Ensure path is valid
  if (!file_exists(image_path)) {
    std::cout << "[send_proof] Could not find file \"" << image_path << "\". Are you sure that's the relative path?" << std::endl;
    return false;
  }

  // Configure cURL request
  CURL *curl = curl_easy_init();
  curl_easy_setopt(curl, CURLOPT_URL, UPLOAD_URL);

  struct curl_httppost *post_begin = NULL;
	struct curl_httppost *post_end = NULL;

  std::string password_hash_str = std::to_string(user_password_hash);
  std::string instruction_id_str = std::to_string(instruction_id);

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

  // Perform cURL
  curl_easy_setopt(curl, CURLOPT_POST, true);
	curl_easy_setopt(curl, CURLOPT_HTTPPOST, post_begin);
	CURLcode code = curl_easy_perform(curl);

  int http_response_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_response_code);
  bool success = http_response_code == 200;

  // Cleanup
  curl_easy_cleanup(curl);

  return success;
}
