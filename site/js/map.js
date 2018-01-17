function sleep(seconds){
    var waitUntil = new Date().getTime() + seconds*1000;
    while(new Date().getTime() < waitUntil) true;
}

var mymap = L.map('mapid').setView([50.607806, 3.136296], 13);

L.tileLayer('http://{s}.tile.osm.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(mymap);
    
var marker = L.marker([50.607806, 3.136296]).addTo(mymap);

sleep(5);

var marker2 = L.marker([50.607808, 3.136298]).addTo(mymap);

marker.bindPopup("Your Gear").openPopup();
