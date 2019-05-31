#ifndef SCAVENGER_HUNT_H
#define SCAVENGER_HUNT_H

#include <curl/curl.h>
#include <json/json.h>

class ScavengerHuntClient {
protected:
  const std::string URL = "http://localhost/script/get_tasks.php";

  CURL* curl;
  std::string *http_received_data;

  static std::size_t curl_write_cb(const char* in, std::size_t size,
      std::size_t num, std::string* out);

public:
  ScavengerHuntClient();

  ~ScavengerHuntClient();

  void retrieve();
};

#endif