<?php
// generate XML TODO: make this actual useful XML based on database
$xml = new SimpleXMLElement('<xml/>');
for ($i = 1; $i <= 8; ++$i) {
        $track = $xml->addChild('task');
            $track->addChild('taskID', "$i");
            $track->addChild('taskType', "unknown");
}
Header('Content-type: text/xml');
print($xml->asXML());
// POST request
//TODO: pick an actual URL to send the data to
$url = "http://ptsv2.com/t/4f403-1550454170/post"; // url to test POST request
$fields = array( // data to send via post request
    'xmlcontent' => $xml->asXML()
);
$ch = curl_init();
curl_setopt($ch,CURLOPT_URL, $url);
curl_setopt($ch,CURLOPT_POST, count($fields));
curl_setopt($ch,CURLOPT_POSTFIELDS, $fields);
//So that curl_exec returns the contents of the cURL; rather than echoing it
curl_setopt($ch,CURLOPT_RETURNTRANSFER, true);
$result = curl_exec($ch);
?>
