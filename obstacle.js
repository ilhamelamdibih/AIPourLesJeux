// Définition de la classe Obstacle
class Obstacle {
  // Constructeur de la classe
  constructor(x, y, r) {
    // Position de l'obstacle
    this.pos = createVector(x, y);
    // Rayon de l'obstacle
    this.r = r;
    // Couleur de l'obstacle (vert par défaut)
    this.color = color(0, 255, 0);
  }

  // Méthode pour afficher l'obstacle
  show() {
    // Sauvegarde de la configuration graphique actuelle
    push();
    // Remplissage avec la couleur de l'obstacle
    fill(this.color);
    // Contour noir
    stroke(0);
    // Épaisseur du contour
    strokeWeight(3);
    // Dessin d'un cercle à la position de l'obstacle avec un diamètre égal à deux fois le rayon
    ellipse(this.pos.x, this.pos.y, this.r * 2);
    // Dessin d'un cercle noir au centre de l'obstacle pour ajouter un détail visuel
    fill(0);
    ellipse(this.pos.x, this.pos.y, 10);
    // Restauration de la configuration graphique précédente
    pop();
  }
}
