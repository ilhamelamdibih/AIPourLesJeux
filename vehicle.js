// Définition de la classe Vehicle
class Vehicle {
  // Propriété statique pour activer ou désactiver le mode débogage
  static debug = false;
  // Constructeur de la classe, initialisation des propriétés
  constructor(x, y) {
    // position du véhicule
    this.pos = createVector(x, y);
    // vitesse du véhicule
    this.vel = createVector(0, 0);
    // accélération du véhicule
    this.acc = createVector(0, 0);
    // vitesse maximale du véhicule
    this.maxSpeed = 4;
    // force maximale appliquée au véhicule
    this.maxForce = 0.4;
    // Rayon pour le dessin du véhicule
    this.r_pourDessin = 10;
    // à peu près en secondes
    this.dureeDeVie = 5;
     // Poids pour le comportement "arrive"
    this.weightArrive = 0.3;
    // Poids pour le comportement "Avoidobstacle"
    this.weightObstacle = 0.9;
    // Rayon du véhicule pour l'évitement
    this.r_pourDessin = 8;
    // rayon du véhicule pour l'évitement
    this.r = this.r_pourDessin * 3;

    // Pour évitement d'obstacle
    this.largeurZoneEvitementDevantVaisseau = 40;
    this.rayonZoneDeFreinage = 200;
    this.perceptionRadius=100;
  }

  // Méthode pour échapper à un autre véhicule
  evade(vehicle) {
    let pursuit = this.pursue(vehicle);
    pursuit.mult(-1);
    return pursuit;
  }

  // Méthode pour poursuivre un autre véhicule
  pursue(vehicle) {
    let target = vehicle.pos.copy();
    let prediction = vehicle.vel.copy();
    prediction.mult(10);
    target.add(prediction);
    fill(0, 255, 0);
    circle(target.x, target.y, 16);
    return this.seek(target);
  }

   // Méthode pour gérer la séparation entre les véhicules
  separation(boids) {
    let steering = createVector();
    let total = 0;
    for (let other of boids) {
      let d = dist(this.pos.x, this.pos.y, other.pos.x, other.pos.y);
      if (other != this && d < this.perceptionRadius) {
        let diff = p5.Vector.sub(this.pos, other.pos);
        diff.div(d * d);
        steering.add(diff);
        total++;
      }
    }
    if (total > 0) {
      steering.div(total);
      steering.setMag(this.maxSpeed);
      steering.sub(this.vel);
      steering.limit(this.maxForce);
    }
    return steering;
  }

  // on fait une méthode applyBehaviors qui applique les comportements
  // seek et avoid
  applyBehaviors(target, obstacles) {

    let arriveForce = this.arrive(target);
    let avoidForce = this.avoidAmeliore(obstacles, vehicles, false);

    // Multiplication par les poids correspondants
    arriveForce.mult(this.weightArrive);
    avoidForce.mult(this.weightObstacle);
    
    // Application des forces résultantes
    this.applyForce(arriveForce);
    this.applyForce(avoidForce);
  }

  // Méthode améliorée pour l'évitement d'obstacle
  avoidAmeliore(obstacles, vehicules, vehiculesAsObstacles = false) {
    // calcul d'un vecteur ahead devant le véhicule
    let ahead = this.vel.copy();
    ahead.normalize();
    ahead.mult(20 * this.vel.mag() * 0.8);

    // Deuxième vecteur deux fois plus petit
    let ahead2 = ahead.copy();
    ahead2.mult(0.5);

    // on les dessine
    if (Vehicle.debug) {
      this.drawVector(this.pos, ahead, "lightblue");
      this.drawVector(this.pos, ahead2, "red");
    }

    // Detection de l'obstacle le plus proche
    let obstacleLePlusProche = this.getObstacleLePlusProche(obstacles);
    let vehiculeLePlusProche = this.getVehiculeLePlusProche(vehicules);

    // On calcule la distance entre le cercle et le bout du vecteur ahead
    let pointAuBoutDeAhead = p5.Vector.add(this.pos, ahead);
    let pointAuBoutDeAhead2 = p5.Vector.add(this.pos, ahead2);

    // On dessine ce point pour debugger
    if (Vehicle.debug) {
      fill("red");
      noStroke();
      circle(pointAuBoutDeAhead.x, pointAuBoutDeAhead.y, 10);

      // On dessine la zone d'évitement
      // On trace une ligne large qui va de la position du vaisseau
      // jusqu'au point au bout de ahead
      stroke(color(255, 200, 0, 90)); // gros, semi transparent
      line(this.pos.x, this.pos.y, pointAuBoutDeAhead.x, pointAuBoutDeAhead.y);
    }
    let distance1 = pointAuBoutDeAhead.dist(obstacleLePlusProche.pos);
    let distance2 = pointAuBoutDeAhead2.dist(obstacleLePlusProche.pos);
    // on tient compte aussi de la position du vaisseau
    let distance3 = this.pos.dist(obstacleLePlusProche.pos);
    let distance4 = Infinity;
    if(vehiculeLePlusProche) {
      distance4 = this.pos.dist(vehiculeLePlusProche.pos);
    } 

    let plusPetiteDistance = min(distance1, distance2);
    plusPetiteDistance = min(plusPetiteDistance, distance3)

    // la plus petite distance est bien celle par rapport à l'obstacle
    // le plus proche

    // point de référence = point au bout de ahead, de ahead2 ou pos
    let pointDeReference;
    if (distance1 < distance2) {
      pointDeReference = pointAuBoutDeAhead;
    } else {
      pointDeReference = pointAuBoutDeAhead2;
    }
    if ((distance3 < distance1) && (distance3 < distance2)) {
      pointDeReference = this.pos;
    }
    // alerte rouge que si vaisseau dans obstacle
    let alerteRougeVaisseauEnCollisionAvecObstacleLePlusProche = (distance3 < obstacleLePlusProche.r);

    // Si le vaisseau n'est pas dans l'obstacle
    // on peut éventuellement considérer le vehicule le plus proche
    // comme l'obstacle à éviter, seulement s'il est plus proche
    // que l'obstacle le plus proche
    if(vehiculesAsObstacles) {
      if (!alerteRougeVaisseauEnCollisionAvecObstacleLePlusProche) {
        let distanceAvecVehiculeLePlusProche = distance4;
        let distanceAvecObstacleLePlusProche = distance3;
  
        if (distanceAvecVehiculeLePlusProche < distanceAvecObstacleLePlusProche) {
          obstacleLePlusProche = vehiculeLePlusProche;
          plusPetiteDistance = distanceAvecVehiculeLePlusProche;
        }
      }
    }
    

    // si la distance est < rayon de l'obstacle le plus proche
    // il y a collision possible et on dessine l'obstacle en rouge
    if (plusPetiteDistance < obstacleLePlusProche.r + this.largeurZoneEvitementDevantVaisseau) {
      // collision possible
      obstacleLePlusProche.color = "red";
      // calcul de la force d'évitement. C'est un vecteur qui va
      // du centre de l'obstacle vers le point au bout du vecteur ahead
      let force = p5.Vector.sub(pointDeReference, obstacleLePlusProche.pos);

      if (Vehicle.debug) {
        // on le dessine pour vérifier qu'il est ok (dans le bon sens etc)
        this.drawVector(obstacleLePlusProche.pos, force, "yellow");
      }

      // Dessous c'est l'ETAPE 2 : le pilotage (comment on se dirige vers la cible)
      // on limite ce vecteur à la longueur maxSpeed
      force.setMag(this.maxSpeed);
      // on calcule la force à appliquer pour atteindre la cible
      force.sub(this.vel);
      // on limite cette force à la longueur maxForce
      force.limit(this.maxForce);

      if (alerteRougeVaisseauEnCollisionAvecObstacleLePlusProche) {
        force.setMag(this.maxForce * 2);
      }
      return force;
    } else {
      // pas de collision possible
      obstacleLePlusProche.color = "green";
      return createVector(0, 0);
    }
  }

   // Méthode pour obtenir l'obstacle le plus proche
  getObstacleLePlusProche(obstacles) {
    let plusPetiteDistance = 100000000;
    let obstacleLePlusProche;

    obstacles.forEach(o => {
      // Je calcule la distance entre le vaisseau et l'obstacle
      const distance = this.pos.dist(o.pos);
      if (distance < plusPetiteDistance) {
        plusPetiteDistance = distance;
        obstacleLePlusProche = o;
      }
    });

    return obstacleLePlusProche;
  }

  // Méthode pour obtenir le véhicule le plus proche
  getVehiculeLePlusProche(vehicules) {
    let plusPetiteDistance = Infinity;
    let vehiculeLePlusProche;

    vehicules.forEach(v => {
      if (v != this) {
        // Je calcule la distance entre le vaisseau et le vehicule
        const distance = this.pos.dist(v.pos);
        if (distance < plusPetiteDistance) {
          plusPetiteDistance = distance;
          vehiculeLePlusProche = v;
        }
      }
    });

    return vehiculeLePlusProche;
  }


  getClosestObstacle(pos, obstacles) {
    // on parcourt les obstacles et on renvoie celui qui est le plus près du véhicule
    let closestObstacle = null;
    let closestDistance = 1000000000;
    for (let obstacle of obstacles) {
      let distance = pos.dist(obstacle.pos);
      if (closestObstacle == null || distance < closestDistance) {
        closestObstacle = obstacle;
        closestDistance = distance;
      }
    }
    return closestObstacle;
  }


  // Méthode pour arriver à une target
  arrive(target) {
    // 2nd argument true enables the arrival behavior
    return this.seek(target, true);
  }

  // Méthode pour fuir une target
  flee(target) {
    return this.seek(target).mult(-1);
  }

  // Méthode pour rechercher une target
  seek(target, arrival = false) {
    let force = p5.Vector.sub(target, this.pos);
    let desiredSpeed = this.maxSpeed;
    
    if (arrival) {

      
      let rayon = this.rayonZoneDeFreinage;

      // - dessiner le cercle de rayon 100 autour du véhicule
      if(Vehicle.debug)
      {
        noFill();
        stroke(255)
        circle(this.pos.x, this.pos.y, rayon);
      }
     


      
      // - calcul de la distance entre la cible et le véhicule
      let distance = p5.Vector.dist(this.pos, target);

      // - si distance < rayon du cercle, alors on modifie desiredSPeed
      // qui devient inversement proportionnelle à la distance.
      // si d = rayon alors desiredSpeed = maxSpeed
      // si d = 0 alors desiredSpeed = 0

      if(distance < rayon) {
        desiredSpeed = map(distance, 0, rayon, 0, this.maxSpeed);
      }
    }

    force.setMag(desiredSpeed);
    force.sub(this.vel);
    force.limit(this.maxForce);
    return force;
  }

  // Méthode pour appliquer une force au véhicle
  applyForce(force) {
    this.acc.add(force);
  }

  // Méthode pour mettre à jour la position, la vitesse, et l'accélération du véhicle
  update() {
    this.vel.add(this.acc);
    this.vel.limit(this.maxSpeed);
    this.pos.add(this.vel);
    this.acc.set(0, 0);
  }

  // Méthode pour afficher le véhicle
  show() {
    stroke(255);
    strokeWeight(2);
    fill(255,0,0);
    push();
    translate(this.pos.x, this.pos.y);
    rotate(this.vel.heading());
    triangle(-this.r_pourDessin, -this.r_pourDessin / 2, -this.r_pourDessin, this.r_pourDessin / 2, this.r_pourDessin, 0);
    pop();
  }

  // Méthode pour gérer les bords de l'écran
  edges() {
    if (this.pos.x > width + this.r) {
      this.pos.x = -this.r;
    } else if (this.pos.x < -this.r) {
      this.pos.x = width + this.r;
    }
    if (this.pos.y > height + this.r) {
      this.pos.y = -this.r;
    } else if (this.pos.y < -this.r) {
      this.pos.y = height + this.r;
    }
  }

   // Méthode pour éviter un autre véhicule
  avoid(vehicule) {
    // calcul d'un vecteur ahead devant le véhicule
    // il regarde par exemple 50 frames devant lui
    let ahead = vehicule.vel.copy();
    ahead.normalize();
    ahead.mult(50);

    if(Vehicle.debug){
    // on le dessine
    this.drawVector(vehicule.pos, ahead, "lightblue");
    }
    

    // On calcule la distance entre le cercle et le bout du vecteur ahead
    let pointAuBoutDeAhead = p5.Vector.add(vehicule.pos, ahead);
    if(Vehicle.debug){

      // On dessine ce point pour debugger
      fill("green");
      noStroke();
      circle(pointAuBoutDeAhead.x, pointAuBoutDeAhead.y, 20);
   

    // On dessine la zone d'évitement
    // On trace une ligne large qui va de la position du vaisseau
    // jusqu'au point au bout de ahead
    stroke(color(10, 120, 0, 30)); // gros, semi transparent
    strokeWeight(1);
    line(vehicule.pos.x, vehicule.pos.y, pointAuBoutDeAhead.x, pointAuBoutDeAhead.y);
  }
    let distance = pointAuBoutDeAhead.dist(vehicule.pos);
    //console.log("distance = " + distance)

    // si la distance est < rayon de l'obstacle
    if (distance < vehicule.r + this.largeurZoneEvitementDevantVaisseau + this.r ) {
      // calcul de la force d'évitement. C'est un vecteur qui va
      // du centre de l'obstacle vers le point au bout du vecteur ahead
      let force = p5.Vector.sub(pointAuBoutDeAhead, vehicule.pos);
      // on le dessine pour vérifier qu'il est ok (dans le bon sens etc)
      if(Vehicle.debug){

      this.drawVector(vehicule.pos, force, "red");
      }
      // Dessous c'est l'ETAPE 2 : le pilotage (comment on se dirige vers la cible)
      // on limite ce vecteur à la longueur maxSpeed
      force.setMag(this.maxSpeed);
      // on calcule la force à appliquer pour atteindre la cible
      force.sub(this.vel);
      // on limite cette force à la longueur maxForce
      force.limit(this.maxForce);
      return force;
    } else {
      // pas de collision possible
      return createVector(0, 0);
    }
  }

  // Méthode pour dessiner un vecteur
  drawVector(pos, v, color) {
    push();
    // Dessin du vecteur vitesse
    // Il part du centre du véhicule et va dans la direction du vecteur vitesse
    strokeWeight(3);
    stroke(color);
    line(pos.x, pos.y, pos.x + v.x, pos.y + v.y);
    // dessine une petite fleche au bout du vecteur vitesse
    let arrowSize = 5;
    translate(pos.x + v.x, pos.y + v.y);
    rotate(v.heading());
    translate(-arrowSize / 2, 0);
    triangle(0, arrowSize / 2, 0, -arrowSize / 2, arrowSize, 0);
    pop();
  }

}


// Définition de la classe Target qui hérite de la classe Vehicle
class Target extends Vehicle {
  constructor(x, y) {
    super(x, y);
    this.vel = p5.Vector.random2D();
    this.vel.mult(5);
  }
  
// Méthode pour afficher la cible
  show() {
    stroke(255);
    strokeWeight(2);
    fill("#F063A4");
    push();
    translate(this.pos.x, this.pos.y);
    circle(0, 0, this.r * 2);
    pop();
  }
}
