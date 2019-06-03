#include <curl/curl.h>
#include <string>
#include <iostream>

static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp) {
	((std::string*)userp)->append((char*)contents, size * nmemb);
	return size * nmemb;
}

int hash(std::string str) {
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

int main(void) {

	std::string url = "http://localhost/script/upload.php";

	CURL* curl = curl_easy_init();

	// set target url
	curl_easy_setopt(curl, CURLOPT_URL, url.c_str());

	// holders
	struct curl_httppost* beginPostList = NULL;
	struct curl_httppost* endPostList = NULL;

	std::string concat1 = "bottle.png";
	std::string email = "aaaaa@aaaaaaa.com";
	std::string password = "sick robots";
	std::string password_hashed = std::to_string(hash(password));
	std::string instr_id = "98";

	curl_formadd(&beginPostList,
	             &endPostList,
							 CURLFORM_COPYNAME, "image",
               CURLFORM_FILE, concat1.c_str(),
							 CURLFORM_END);
	curl_formadd(&beginPostList,
               &endPostList,
             	 CURLFORM_COPYNAME, "email",
             	 CURLFORM_COPYCONTENTS, email.c_str(),
             	 CURLFORM_END);
	curl_formadd(&beginPostList,
	             &endPostList,
	          	 CURLFORM_COPYNAME, "pass_hash",
	          	 CURLFORM_COPYCONTENTS, password_hashed.c_str(),
	          	 CURLFORM_END);
	 curl_formadd(&beginPostList,
	 						 &endPostList,
	 						 CURLFORM_COPYNAME, "instr_id",
	 						 CURLFORM_COPYCONTENTS, instr_id.c_str(),
	 						 CURLFORM_END);

	// perform
	curl_easy_setopt(curl, CURLOPT_POST, true);
	curl_easy_setopt(curl, CURLOPT_HTTPPOST, beginPostList);
	CURLcode code = curl_easy_perform(curl);
}
