#include "scavenger/scavenger_hunt.h"

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

std::size_t ScavengerHuntClient::curl_write_cb(const char* in, std::size_t size,
    std::size_t num, std::string* out) {
  const std::size_t total_bytes(size * num);
  out->append(in, total_bytes);
  return total_bytes;
}

ScavengerHuntClient::ScavengerHuntClient() {
  curl = curl_easy_init();

  curl_easy_setopt(curl, CURLOPT_URL, URL.c_str());
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10); // Time out after 10 seconds
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L); // Allow 1 redirect
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,
      ScavengerHuntClient::curl_write_cb);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, http_received_data);
}

ScavengerHuntClient::~ScavengerHuntClient() {}

void ScavengerHuntClient::retrieve() {
  int http_response_code = 0;

  curl_easy_perform(curl);
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_response_code);
  curl_easy_cleanup(curl);

  if (http_response_code == 200) {
    std::cout << "[retrieve] Got response from server. Parsing..." << std::endl;

    std::stringstream parser_stream;
    parser_stream << http_received_data->c_str();

    Json::Value json_data;
    Json::CharReaderBuilder json_reader;
    std::string errs;

    Json::parseFromStream(json_reader, parser_stream, &json_data, &errs);

    parser_stream.seekg(0, std::iostream::end);
    int size = parser_stream.tellg();
    parser_stream.seekg(0, std::iostream::beg);

    std::cout << "[retrieve] Successfully parsed " << size << " characters of JSON data" << std::endl;
  } else {
    std::cout << "[retrieve] Failed to contact server" << std::endl;
  }
}