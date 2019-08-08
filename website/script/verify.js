var proofIds = [];

// Open the Modal
function openModal(type, fname) {
  document.getElementById("myModal").style.display = "block";

  var container = document.getElementById("lightbox-slide-" + type);
  container.setAttribute("src", fname);
  container.style.display = "block";

  if (type == "video") {
    container.setAttribute("width", "640");
    container.setAttribute("height", "480");
  }
}

// Close the Modal
function closeModal() {
  document.getElementById("myModal").style.display = "none";
  document.getElementById("lightbox-slide-image").setAttribute("src", "");
  document.getElementById("lightbox-slide-video").setAttribute("src", "");
  document.getElementById("lightbox-slide-image").style.display = "none";
  document.getElementById("lightbox-slide-video").setAttribute("width", "0");
  document.getElementById("lightbox-slide-video").setAttribute("height", "0");
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
