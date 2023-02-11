using LinearAlgebra

mutable struct KalmanFilter
	x::Matrix{Float64}
	P::Matrix{Float64}
	Q::Matrix{Float64}
	R::Matrix{Float64}
	F::Matrix{Float64}
	H::Matrix{Float64}
	K::Matrix{Float64}

	function KalmanFilter(xdim::Int64, zdim::Int64)
		new(zeros(Float64, xdim, 1), Matrix{Float64}(I, xdim, xdim), Matrix{Float64}(I, xdim, xdim), Matrix{Float64}(I, zdim, zdim), zeros(Float64, xdim, xdim), zeros(Float64, zdim, xdim), zeros(Float64, xdim, zdim))
	end
end

function reset(kf::KalmanFilter)
	fill!(kf.x, 0)
	kf.P = Matrix{Float64}(I, size(kf.P)[1], size(kf.P)[2])
	fill!(kf.K, 0)
end

function predict(kf::KalmanFilter)
	kf.x = kf.F * kf.x
	kf.P = kf.F * kf.P * kf.F' + kf.Q
end

function update(kf::KalmanFilter, z::Matrix{Float64})
	y = z - kf.H * kf.x
	tS = kf.H * kf.P * kf.H' + kf.R
	kf.K = kf.P * kf.H' * inv(tS)
	kf.x = kf.x + kf.K * y
	kf.P = kf.P - kf.K * kf.H * kf.P
end