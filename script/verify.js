//parameters: src is the filepath to the image
function show_image(src){
  var img = document.createElement("img");
  img.src = src;
  img.alt = "cannot display image";
  // add image to page
  document.body.appendChild(img);
}


function updateTable(proofId){
  // the proof is correct if user inputs "yes"
  if(document.getElementById('check_yes').checked){
    $.ajax({
      type: "POST",
      url: '../script/set_correct.php',
      data: {proofId: proofId},

      success: function(output) {
        console.log("works!");
      },
      error: function(request, status, error){
        alert("Error: " + error);
        console.log("it wrong :(");
      }
    });
  }
  // the proof has been verified by the user, update table
  $.ajax({
    type: "POST",
    url: '../script/set_verified.php',
    data: {proofId: proofId},

    success: function(output) {
      console.log("verification works!");
    },
    error: function(request, status, error){
      alert("Error: " + error);
      console.log("verification is wrong :(");
    }
  });
  document.getElementById("check_yes").disabled = true;
  document.getElementById("check_no").disabled = true;
  // document.getElementById("submit").disabled = true;
  document.getElementById("submit").style.visibility = "hidden";
}
