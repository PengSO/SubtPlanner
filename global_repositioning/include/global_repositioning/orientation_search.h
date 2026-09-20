#pragma once

#include "global_repositioning/nelder_mead_rot.h"

#include <boost/function.hpp>
#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>

struct OrientationSearchParams {
  int max_iterations = 100;
  int max_iter_nm = 20;
  int population_size = 10;
  double tol = 1e-2;
  int tol_iter = 5;
};

class Individual {
  public:
  Individual(){};
  Individual(bool random);
  Eigen::Matrix3d& operator[](int i) { return chromosome_[i]; };
  double calculateFitness(const boost::function<double(const Eigen::Matrix3d)>& objFun);

  void mutation(const boost::function<double(const Eigen::Matrix3d)>& objFun,
                NelderMeadRot<Eigen::Matrix3d>& nealder_mead);

  double getFitness() const { return fitness_; };

  int getFitnessIndex() const { return fitness_idx_; };
  Eigen::Matrix3d& getFittestGene() { return chromosome_[fitness_idx_]; };

  private:
  double fitness_;
  int fitness_idx_;
  std::vector<Eigen::Matrix3d> chromosome_;
};

class OrientationSearch {
  public:
  OrientationSearch(){};
  OrientationSearch(OrientationSearchParams params) : params_(params), nealder_mead_(params.max_iter_nm){};

  void setParams(OrientationSearchParams params) { params_ = params; };

  void initPopulation(std::vector<Individual>& population);
  double calculateFitness(std::vector<Individual>& population);
  void sortIndividuals(std::vector<Individual>& population);
  void crossover(std::vector<Individual>& population);
  void mutation(std::vector<Individual>& population);

  Eigen::Matrix3d runOrientationSearch(const boost::function<double(const Eigen::Matrix3d)> objFun);

  private:
  // Fitness function
  boost::function<double(const Eigen::Matrix3d)> objFun_;
  std::vector<Individual> population_;
  OrientationSearchParams params_;
  NelderMeadRot<Eigen::Matrix3d> nealder_mead_;
};
