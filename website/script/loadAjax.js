$(document).ready(function (){
       var loadCount = 0;
       $("#plusButton").click(function (event){
		loadCount = loadCount + 1;
		$('#loadData').load("../script/taskupdating.php",{
			loadNewCount: loadCount
					
		});				
	});
});
		 
		 
	
