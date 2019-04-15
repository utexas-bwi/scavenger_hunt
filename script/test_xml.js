function testXML() {
    console.log("send get request");
    const url = "http://localhost/script/get_tasks.php";
    $.ajax({
	robot_tasks: true,
	type: "GET",
	url,
	success: function(data) {
	    console.log("done");
	    console.log(data);
	}
    });
}
