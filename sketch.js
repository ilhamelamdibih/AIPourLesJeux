// Déclaration des variables globales
let vehicle  
let target  
let follow;
let vehicles=[];
let obstacles = [];
let sliderVitesseMaxCible;

function setup() {

  // Initialisation du canvas pour prondre la taille de l'écran
  createCanvas(windowWidth, windowHeight);

  // Initialisation du slider pour la vitesse initial de 6
  sliderVitesseMaxCible = createSlider(1, 10, 6, 0.1);

  // On cree 5 vehicles avec des positions aléatoires
   for(let i =0;i<5;i++){
    let v= new Vehicle(random(width),random(height));
    v.maxSpeed=5;
    v.maxForce=2;
    vehicles.push(v);
   }

  // On cree un obstalce au milieu de l'écran
  obstacle = new Obstacle(width / 2, height / 2, 50);
  obstacles.push(obstacle);

}

function draw() {
  // changer le dernier param (< 100) pour effets de trainée
  background(0, 0, 0, 100);

  // Cible qui suit la souris, cercle blue de rayon 20
  target = createVector(mouseX, mouseY);
  fill(0,0,255);
  noStroke();
  ellipse(target.x, target.y, 20);

  // dessin des obstacles
  obstacles.forEach(o => {
    o.show();
  });

  // dessin des vehicules
  for(let i =0;i<vehicles.length;i++){
    let v= vehicles[0].vel.copy();
    v.normalize();
    v.mult(-100);
    v.add(vehicles[0].pos);
    fill(0,255,0);
    circle(v.x,v.y,15);

    // Ajustement de la vitesse maximale des véhicules selon la valeur du slider
    vehicles[i].maxSpeed = sliderVitesseMaxCible.value();

    // Application des comportements pour le premier véhicule (Leader)
    if(i===0)
    {
      vehicles[i].applyBehaviors(target, obstacles, vehicles);
      let steering =vehicles[i].arrive(target);
      vehicles[i].applyForce(steering);
    }

    // Application des comportements pour les autres véhicules
    else{
    let avoidForce = vehicles[i].avoid(vehicles[0]);
    avoidForce.mult(0.3)
    vehicles[i].applyForce(avoidForce);
    
    // pour la séparation des vehicles
    let separation = vehicles[i].separation(vehicles);
      separation.mult(0.2);
      vehicles[i].applyForce(separation);
      let steering =vehicles[i].arrive(v);
      steering.mult(0.6);
      vehicles[i].applyForce(steering);
     
      let vehiculePrecedent = vehicles[i-1];
      // On prend la vitesse du précédent et on en fait une copie
      let pointDerriere = vehiculePrecedent.vel.copy();
      // on le normalise
      pointDerriere.normalize();
      // et on le multiplie par une distance derrière le vaisseau
      pointDerriere.mult(-30);
      // on l'ajoute à la position du vaisseau
      pointDerriere.add(vehiculePrecedent.pos);

      //pour éviter les obstacles
      vehicles[i].applyBehaviors(pointDerriere, obstacles, vehicles);
     
    }
   
    // Mise à jour et affichage des véhicules
    vehicles[i].update();
    vehicles[i].show();
   }
 

   
}

// Fonction appelée lors du clic de la souris
function mousePressed() {
   // Création d'un nouvel obstacle à la position de la souris avec un rayon aléatoire
  obstacle = new Obstacle(mouseX, mouseY, random(5, 60));
  obstacles.push(obstacle);
}

// Fonction appelée lors de l'appui sur une touche du clavier
function keyPressed() {
  // Ajout d'un nouveau véhicule avec une position aléatoire lors de l'appui sur la touche "v"
  if (key == "v") {
    vehicles.push(new Vehicle(random(width), random(height)));
  }
  // Inversion du mode de débogage lors de l'appui sur la touche "d"
  if (key == "d") {
    Vehicle.debug = !Vehicle.debug;
  }
}
