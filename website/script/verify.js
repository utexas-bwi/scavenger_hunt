// //parameters: src is the filepath to the image
// function show_image(src){
//   var img = document.createElement("img");
//   img.src = src;
//   img.alt = "cannot display image";
//   // add image to page
//   document.body.appendChild(img);
// }
//
// // called when user presses submit for the question
// // parameters: the imageFilename, proofId, hunt_intsr_id, and uploaderId associated with the
// // question in which the user has pressed submit for. These are accessed from the proof_table in verify.php
// function updateTable(imageFilename, proofId, huntInstrId, uploaderId){
//
//   // the proof is correct if user inputs "yes", sets correct to be true in table
//   if(document.getElementById(imageFilename).checked){
//     $.ajax({
//       type: "POST",
//       url: '../script/set_correct.php',
//       data: {proofId: proofId, huntInstrId: huntInstrId, uploaderId: uploaderId},
//
//       success: function(output) {
//         console.log("its correct");
//       },
//       error: function(request, status, error){
//         alert("Error: " + error);
//         console.log("setting correct failed :(");
//       }
//     });
//   }
//
//   // the proof has been verified by the user, sets verified to be true in table
//   $.ajax({
//     type: "POST",
//     url: '../script/set_verified.php',
//     data: {proofId: proofId},
//
//     success: function(output) {
//       console.log("verification works!");
//     },
//     error: function(request, status, error){
//       alert("Error: " + error);
//       console.log("verification is wrong :(");
//     }
//   });
//
//   // disables the radio inputs (yes and no)
//   var x = document.getElementsByName(imageFilename);
//   for (var i = 0; i < x.length - 1; i++)
//     x[i].disabled = true;
//
//   // sets last input (submit button) to be hidden
//   x[x.length -1].style.visibility = "hidden";
// }
//
// function enableSubmit(proofId){
//   document.getElementById(proofId).disabled = false;
//   document.getElementById(proofId).style.backgroundColor = "transparent";
// }

var proofIds = [];

// Open the Modal
function openModal(fname) {
  document.getElementById("myModal").style.display = "block";
  document.getElementById("lightbox-slide-image").setAttribute("src", fname);
}

// Close the Modal
function closeModal() {
  document.getElementById("myModal").style.display = "none";
}

function submitProofValidations() {
  var validations = document.getElementsByClassName("validation");
  var validationCount = 0;

  for (var i = 0; i < validations.length; i++) {
    var v = validations[i];
    var buttons = v.getElementsByTagName("input");
    var correctButton = buttons[0];
    var incorrectButton = buttons[1];
    var status = "unvalidated";

    if (correctButton.checked)
      status = "correct";
    else if (incorrectButton.checked)
      status = "incorrect";

    if (status != "unvalidated") {
      correctButton.disabled = true;
      incorrectButton.disabled = true;
      validationCount++;

      var proofId = v.getAttribute("proof_id");
      var huntInstrId = v.getAttribute("hunt_instr_id");
      var uploaderId = v.getAttribute("uploader_id");

      if (status == "correct") {
        $.ajax({
          type: "POST",
          url: '../script/set_correct.php',
          data: {proofId: proofId, huntInstrId: huntInstrId, uploaderId: uploaderId}
        });
      }

      $.ajax({
        type: "POST",
        url: '../script/set_verified.php',
        data: {proofId: proofId}
      });
    }
  }

  if (validationCount > 0) {
    const message = document.getElementById("save-msg");
    message.classList.remove("hidden-msg");
    message.classList.remove("alert-danger");
    message.classList.add("alert-success");
    message.textContent = "Successfully verified " + validationCount + " proof(s).";
    setTimeout(function() {
        message.classList.add("hidden-msg");
        message.classList.remove("alert-danger");
    }, 4000);
  }
}
