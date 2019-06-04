//parameters: src is the filepath to the image
function show_image(src){
  var img = document.createElement("img");
  img.src = src;
  img.alt = "cannot display image";
  // add image to page
  document.body.appendChild(img);
}

// called when user presses submit for the question
// parameters: the imageFilename, proofId, hunt_intsr_id, and uploaderId associated with the
// question in which the user has pressed submit for. These are accessed from the proof_table in verify.php
function updateTable(imageFilename, proofId, huntInstrId, uploaderId){

  // the proof is correct if user inputs "yes", sets correct to be true in table
  if(document.getElementById(imageFilename).checked){
    $.ajax({
      type: "POST",
      url: '../script/set_correct.php',
      data: {proofId: proofId, huntInstrId: huntInstrId, uploaderId: uploaderId},

      success: function(output) {
        console.log("its correct");
      },
      error: function(request, status, error){
        alert("Error: " + error);
        console.log("setting correct failed :(");
      }
    });
  }
  
  // the proof has been verified by the user, sets verified to be true in table
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

  // disables the radio inputs (yes and no)
  var x = document.getElementsByName(imageFilename);
  for (var i = 0; i < x.length - 1; i++) 
    x[i].disabled = true;
  
  // sets last input (submit button) to be hidden
  x[x.length -1].style.visibility = "hidden";
}
