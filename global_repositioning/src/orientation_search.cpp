
#include "global_repositioning/orientation_search.h"

#include <limits>

Individual::Individual(bool random)
{
  for (int i = 0; i < 4; i++) {
    if (random) {
      Eigen::Matrix3d gene = Eigen::Matrix3d::Random(3, 3);
      gene = gene.householderQr().householderQ();
      chromosome_.push_back(gene);
    }
  }
}

double Individual::calculateFitness(const boost::function<double(const Eigen::Matrix3d)>& objFun)
{
  fitness_ = std::numeric_limits<double>::max();
  fitness_idx_ = 0;

  int idx = 0;
  for (auto& gene : chromosome_) {
    double fitness_gene = objFun(gene);
    if (fitness_gene < fitness_) {
      fitness_ = fitness_gene;
      fitness_idx_ = idx;
    }

    idx = idx + 1;
  }
  return fitness_;
}

void Individual::mutation(const boost::function<double(const Eigen::Matrix3d)>& objFun,
                          NelderMeadRot<Eigen::Matrix3d>& nealder_mead)
{
  nealder_mead.solve(objFun, chromosome_);
}

void OrientationSearch::initPopulation(std::vector<Individual>& population)
{
  population.clear();
  const bool random = true;

  for (int i = 0; i < params_.population_size; i++) {
    population.push_back(Individual(random));
  }
}

double OrientationSearch::calculateFitness(std::vector<Individual>& population)
{
  double best_fitness = std::numeric_limits<double>::max();
  for (auto& individual : population) {
    double fitness = individual.calculateFitness(objFun_);
    if (fitness < best_fitness)
      best_fitness = fitness;
  }
  return best_fitness;
}

void OrientationSearch::sortIndividuals(std::vector<Individual>& population)
{
  stable_sort(population.begin(), population.end(),
              [](const Individual& i1, const Individual& i2)
              {
                return i1.getFitness() < i2.getFitness();
              });
}

void OrientationSearch::crossover(std::vector<Individual>& population)
{
  int population_size_half_c = ceil(params_.population_size / 2);
  int population_size_half_f = floor(params_.population_size / 2);

  std::vector<Individual> pop1;
  std::vector<Individual> pop2;
  std::vector<Individual> pop3;
  std::vector<Individual> pop4;

  for (int i = 0; i < population_size_half_f; i++) {
    int idx1 = rand() % population_size_half_c;
    int idx2 = rand() % population_size_half_c;
    pop1.push_back(population[idx1]);
    pop2.push_back(population[idx2]);
  }

  for (int i = 0; i < population_size_half_c; i++) {
    int idx3 = rand() % population_size_half_f;
    int idx4 = rand() % population_size_half_f;
    pop3.push_back(population[idx3]);
    pop4.push_back(population[idx4]);
  }

  for (int i = 0; i < population_size_half_f; i++) {
    double cutoff = 0.5 + 0.1 * (pop1[i].getFitness() <= pop2[i].getFitness()) -
                    0.1 * (pop1[i].getFitness() >= pop2[i].getFitness());
    for (int j = 0; j < 4; j++) {
      if ((double)rand() / RAND_MAX < cutoff)
        population[i][j] = pop1[i][j];
      else
        population[i][j] = pop2[i][j];
    }
  }

  for (int i = 0; i < population_size_half_c; i++) {
    int i2 = population_size_half_f + i;
    double cutoff = 0.5 + 0.1 * (pop3[i].getFitness() <= pop4[i].getFitness()) -
                    0.1 * (pop3[i].getFitness() >= pop4[i].getFitness());
    for (int j = 0; j < 4; j++) {
      Eigen::Matrix3d mat = cutoff * pop3[i][j] + (1 - cutoff) * pop4[i][j];
      mat = mat.householderQr().householderQ();
      population[i2][j] = mat;
    }
  }
}

void OrientationSearch::mutation(std::vector<Individual>& population)
{
  for (int i = 0; i < params_.population_size; i++) {
    population[i].mutation(objFun_, nealder_mead_);
  }
}

Eigen::Matrix3d OrientationSearch::runOrientationSearch(
    const boost::function<double(const Eigen::Matrix3d)> objFun)
{
  objFun_ = objFun;

  initPopulation(population_);

  double best_fitness = std::numeric_limits<double>::max();
  double old_fitness;
  int tol_counter = 0;

  for (int iter = 0; iter < params_.max_iterations; iter++) {
    old_fitness = best_fitness;
    best_fitness = calculateFitness(population_);

    sortIndividuals(population_);

    if (abs(best_fitness - old_fitness) < params_.tol) {
      tol_counter = tol_counter + 1;
      if (tol_counter >= params_.tol_iter) {
        break;
      }
    }
    else {
      tol_counter = 0;
    }

    crossover(population_);
    mutation(population_);
  }

  return population_[0].getFittestGene();
}
