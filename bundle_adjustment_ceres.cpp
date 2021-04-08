#include <iostream>
#include <ceres/ceres.h>
#include "common.h"
#include "ReprojectionError.h"

using namespace std;

void SolveBA(BALProblem &bal_problem);

int main(int argc, char **argv)
{
	if (argc != 2)
	{
		cout << "usage: ./BA_ceres ../bal_data.txt" << endl;
		return 1;
	}

	BALProblem bal_problem(argv[1]);
	bal_problem.Normalize();
	bal_problem.Perturb(0.1, 0.5, 0.5);
	bal_problem.WriteToPLYFile("initial.ply");
	SolveBA(bal_problem);
	bal_problem.WriteToPLYFile("final.ply");
	return 0;
}

void SolveBA(BALProblem &bal_problem)
{
	const int point_block_size = bal_problem.point_block_size();
	const int camera_block_size = bal_problem.camera_block_size();
	double *points = bal_problem.mutable_points();
	double *cameras = bal_problem.mutable_cameras();

	const double *observations = bal_problem.observations();
	ceres::Problem problem;

	for (int i=0; i<bal_problem.num_observations(); i++)
	{
		ceres::CostFunction *cost_function;

		cost_function = ReprojectionError::Create(observations[2*i + 0], observations[2*i + 1]);
		ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);

		double *camera = cameras + camera_block_size * bal_problem.camera_index()[i];
		double *point = points + point_block_size * bal_problem.point_index()[i];

		problem.AddResidualBlock(cost_function, loss_function, camera, point);
	}

	cout << "BAL problem has " << bal_problem.num_cameras() << " cameras and " 
			<< bal_problem.num_points() << " points" << endl;
	cout << " which forms " << bal_problem.num_observations() << " observations " << endl;

	cout << "Solving BA using ceres " << endl;

	ceres::Solver::Options options;
	options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	cout << summary.FullReport() << endl;
}