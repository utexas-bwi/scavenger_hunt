function submitTask() {
    // remove form and add thank you message
    const content = document.getElementById("content");
    content.removeChild(document.getElementById("task_form"));
    const par = document.createElement("h3");
    par.textContent = "Thank you for submitting a task for the robot!";
    content.appendChild(par);
    // reload page
    setTimeout(location.reload.bind(location), 1000);
    // TODO: add code for storing information from form
}
 
